 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

/*
 *      Watchdog - derived from Roadrunner Watchdog
 *	Written by: David Stavens - Stanford University
 */

#include <carmen/carmen.h>
#include <sys/wait.h>

static pid_t watchdog_pid = -1, spawned_pid = -1;
static char *spawned_name = NULL;

/* Handle pretty much any signal the WATCHDOG receives by killing 
   the child process and exiting. */

void watchdog_handle_signal(int sig __attribute__ ((unused)))
{
  fprintf(stderr, "WATCHDOG (%d): Watchdog was signaled."
	  "Killing %s (%d) and exiting.\n", 
	  watchdog_pid, spawned_name, spawned_pid);
  if(spawned_pid != -1)
    kill(spawned_pid, SIGKILL);
  exit(0);
}

int main(int argc, char **argv)
{
  char *arg_run[4];
  int status, this_signal;
  int this_arg, length_needed = 0;

  if(argc < 2)
    carmen_die("Error: Usage: %s program [arg1] [arg2] ...\n", argv[0]);
	
  spawned_name = argv[1];
  watchdog_pid = getpid();
  
  /* First argument will be "sh" plus one byte for the terminating null. */
  arg_run[0] = (char *)malloc((strlen("tcsh") + 1) * sizeof(char));
  carmen_test_alloc(arg_run[0]);
  strcpy(arg_run[0], "tcsh");
  
  /* Second argument will be "-c" plus one byte for the terminating null. */
  arg_run[1] = (char *)malloc((strlen("-c") + 1) * sizeof(char));
  carmen_test_alloc(arg_run[1]);
  strcpy(arg_run[1], "-c");
  
  /* Fourth argument will be NULL as required by the function. */
  arg_run[3] = NULL;
  
  /* Third argument needs to be a single string with the actual command 
     to run and its arguments. */
  
  /* So, first, count up the length of this third argument. 
     Plus one byte for the space after this argument 
     plus one byte for \0 */
  for(this_arg = 1; this_arg < argc; this_arg++)
    length_needed += strlen(argv[this_arg]) + 1;	
  length_needed += 1;
  
  /* Second, allocate the memory for this third argument. */
  arg_run[2] = (char *)malloc(length_needed * sizeof(char));
  carmen_test_alloc(arg_run[2]);

  /* Initialize first byte to zero so string concatentation works. */
  arg_run[2][0] = '\0';
  
  /* Finally, populate this third argument. */
  for(this_arg = 1; this_arg < argc; this_arg++) {
    strcat(arg_run[2], argv[this_arg]);
    strcat(arg_run[2], " ");
  }
  
  while(1) {
    /* Cycle through all signals.  Max in /usr/include/bits/signum.h is 65. 
       Do more for safety.
       Must do this before forking so child process uses default
       (not WATCHDOG) signal handling. */	
    for(this_signal = 0; this_signal < 128; this_signal++)
      signal(this_signal, SIG_DFL);
    
    /* Actually fork the program as a new process.
       Program splits in two here.  Child process gets 0 returned.  
       Parent gets PID of child. */
    if((spawned_pid = fork()) == 0) {
      /* This assumes that ARGV array is null terminated!!  VERIFY!?! */
      /* execv() replaces the child process image with the 
	 program-to-be-run's image. */
      execv("/bin/sh", arg_run);
      /* execv() only returns if there's an error.  If there's an error, 
	 we just exit uncleanly and the WATCHDOG parent will restart the
	 process again. */
      exit(-1);
    }
    
    /* Cycle through all signals again.
       Now that child is spawned, set WATCHDOG signals the way we want them.
     */
    for(this_signal = 0; this_signal < 128; this_signal++)
      if(this_signal != SIGCHLD && // this_signal != SIGCLD && 
	 this_signal != SIGCONT)
	signal(this_signal, watchdog_handle_signal);
    /* Note: Don't register handler for child signals.  Handler exits on signal
       and we certainly don't want WATCHDOG to exit when child is signaled!! */
    /* Note: Don't register handler for un-suspend signals.  Can't catch 
       suspend signals so no reason to exit on unsuspend signals.  Besides,
       signals from the child buffer up while the WATCHDOG is suspended. */
    
    fprintf(stderr, "WATCHDOG (%d): Spawned %s (%d)\n", watchdog_pid,
	    spawned_name, spawned_pid);
    
    /* Go to sleep until something happens with the child process (ie: if it
       exits or is signaled). */
    waitpid(spawned_pid, &status, WUNTRACED);
    
    /* Check if the child process terminated due to uncaught signal.  
       If so, restart it. */
    if(WIFSIGNALED(status)) {
      fprintf(stderr,
	      "WATCHDOG (%d): %s (%d) exited due to SIGNAL (code = %d).\n", 
	      watchdog_pid, spawned_name, spawned_pid, WTERMSIG(status));
      /* Go to top of loop and start over. */
      continue;
    }
    
    /* Check if the child process was stopped or suspended.  If so, kill 
       it and restart it. */
    if(WIFSTOPPED(status)) {
      fprintf(stderr, "WATCHDOG (%d): %s (%d) was STOPPED "
	      "(code = %d).  Killing process.\n", watchdog_pid, 
	      spawned_name, spawned_pid, WSTOPSIG(status));
      /* Kill the child process. */
      kill(spawned_pid, SIGKILL);
      /* Go to top of loop and start over. */
      continue;
    }
    
    /* Check if the child process exited (ie: return; exit(int); etc.).  
       If so and the code != 0, restart it. */
    if(WIFEXITED(status)) {
      /* If the child exited with code == 0, it wanted to terminate.
	 WATCHDOG quits. */
      if(WEXITSTATUS(status) == 0) {
	fprintf(stderr, "WATCHDOG (%d): %s (%d) exited CLEANLY."
		"  Terminating watchdog.\n", watchdog_pid, spawned_name,
		spawned_pid);
	/* WATCHDOG quits. */
	exit(0);
      }
      /* ...otherwise restart the child. */
      else {
	fprintf(stderr, 
		"WATCHDOG (%d): %s (%d) exited UNCLEANLY (code = %d).\n", 
		watchdog_pid, spawned_name, spawned_pid, WEXITSTATUS(status));
	/* Go to top of loop and start over. */
	continue;
      }
    }
  }
  return 0;	
}
