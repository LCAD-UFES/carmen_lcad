#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include "ipc.h"

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

typedef struct {
  
  int                           nr;
  double                        val;
  
} carmen_test_1_message_type;

#define CARMEN_TEST_1_MESSAGE_NAME       "carmen_test_1_message"

#define CARMEN_TEST_1_MESSAGE_FMT        "{ int, double }"

typedef struct {
  
  int                           nr;
  carmen_test_1_message_type    data;
  
} carmen_test_2_message_type;

#define CARMEN_TEST_2_MESSAGE_NAME       "carmen_test_2_message"

#define CARMEN_TEST_2_MESSAGE_FMT        "{ int, { int, double } }"

typedef struct {
  
  int                           nr;
  carmen_test_1_message_type  * data;
  
} carmen_test_3_message_type;

#define CARMEN_TEST_3_MESSAGE_NAME       "carmen_test_3_message"

#define CARMEN_TEST_3_MESSAGE_FMT        "{ int, < { int, double } : 1 > }"

typedef struct {
  
  int                           len;
  float                       * data;
  int                           nr;
  double                        val;
  
} carmen_test_4_message_type;

#define CARMEN_TEST_4_MESSAGE_NAME       "carmen_test_4_message"

#define CARMEN_TEST_4_MESSAGE_FMT        "{ int, < float : 1 >, int, double }"

typedef struct {
  
  carmen_test_1_message_type    data[5];
  
} carmen_test_5_message_type;

#define CARMEN_TEST_5_MESSAGE_NAME       "carmen_test_5_message"

#define CARMEN_TEST_5_MESSAGE_FMT        "{ [ { int, double } : 5 ] }"


/**************************************************************************/
/*                                                                        */
/**************************************************************************/

#define EPSILON                0.00000001

#define NUM_CHECKS        5

double check[NUM_CHECKS] =
  { 0.0, EPSILON, -EPSILON, 1000.0, -1000.0 };

void ipc_send_test( int type, int nr, double val );
void ipc_initialize_messages( int sender );
void ipc_update( void );
void ipc_init( char * name, int sender );
void ipc_stop( void );
void print_usage( void );


/**************************************************************************/
/*   TEST 1                                                               */
/**************************************************************************/

static void 
ipc_test_1_handler( MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		    void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE                      err = IPC_OK;
  FORMATTER_PTR                        formatter;
  unsigned char                        bytes[8];
  int                                  i;
  static carmen_test_1_message_type    data;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &data,
			   sizeof(carmen_test_1_message_type));
  IPC_freeByteArray(callData);
  if (err != IPC_OK) {
    fprintf(stderr, "Could not unmarshall %s\n", 
	    IPC_msgInstanceName(msgRef));
    return;
  }

  if (data.nr>=NUM_CHECKS) {
    fprintf( stderr, "ERROR: idx number (%d) too big!",
	     data.nr );
  } else {
    fprintf( stderr, "CHECK (t:%d/n:%d): %4.8f  .... : ",
	     1, data.nr, check[data.nr] );
    if (check[data.nr] == data.val) {
      fprintf( stderr, "ok.\n" );
    } else {
      fprintf( stderr, "failed!!!\n" );
      fprintf( stderr, "HERE: "  );
      bcopy( &check[data.nr], bytes, 8 );
      for (i=0;i<8;i++) {
	fprintf( stderr, "[0x%s%x]", bytes[i]<16?"0":"", bytes[i] );
      }
      fprintf( stderr, "\n"  );
      fprintf( stderr, "IPC: "  );
      bcopy( &data.val, bytes, 8 );
      for (i=0;i<8;i++) {
	fprintf( stderr, "[0x%s%x]", bytes[i]<16?"0":"", bytes[i] );
      }
      fprintf( stderr, "\n"  );
    }
    
  }

}

/**************************************************************************/
/*   TEST 2                                                               */
/**************************************************************************/

static void 
ipc_test_2_handler( MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		    void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE                      err = IPC_OK;
  FORMATTER_PTR                        formatter;
  unsigned char                        bytes[8];
  int                                  i;
  static carmen_test_2_message_type    data;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &data,
			   sizeof(carmen_test_2_message_type));
  IPC_freeByteArray(callData);
  if (err != IPC_OK) {
    fprintf(stderr, "Could not unmarshall %s\n", 
	    IPC_msgInstanceName(msgRef));
    return;
  }

  if (data.nr>=NUM_CHECKS) {
    fprintf( stderr, "ERROR: idx number (%d) too big!",
	     data.nr );
  } else {
    fprintf( stderr, "CHECK (t:%d/n:%d): %4.8f  .... : ",
	     2, data.data.nr, check[data.data.nr] );
    if (check[data.data.nr] == data.data.val) {
      fprintf( stderr, "ok.\n" );
    } else {
      fprintf( stderr, "failed!!!\n" );
      fprintf( stderr, "HERE: "  );
      bcopy( &check[data.data.nr], bytes, 8 );
      for (i=0;i<8;i++) {
	fprintf( stderr, "[0x%s%x]", bytes[i]<16?"0":"", bytes[i] );
      }
      fprintf( stderr, "\n"  );
      fprintf( stderr, "IPC: "  );
      bcopy( &data.data.val, bytes, 8 );
      for (i=0;i<8;i++) {
	fprintf( stderr, "[0x%s%x]", bytes[i]<16?"0":"", bytes[i] );
      }
      fprintf( stderr, "\n"  );
    }
    
  }

}

/**************************************************************************/
/*   TEST 3                                                               */
/**************************************************************************/

static void 
ipc_test_3_handler( MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		    void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE                      err = IPC_OK;
  FORMATTER_PTR                        formatter;
  unsigned char                        bytes[8];
  int                                  i;
  static carmen_test_3_message_type    data;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &data,
			   sizeof(carmen_test_3_message_type));
  IPC_freeByteArray(callData);
  if (err != IPC_OK) {
    fprintf(stderr, "Could not unmarshall %s\n", 
	    IPC_msgInstanceName(msgRef));
    return;
  }

  if (data.nr>=NUM_CHECKS) {
    fprintf( stderr, "ERROR: idx number (%d) too big!\n",
	     data.data[0].nr );
  } else {
    fprintf( stderr, "CHECK (t:%d/n:%d): %4.8f  .... : ",
	     3, data.data[0].nr, check[data.data[0].nr] );
    if (check[data.data[0].nr] == data.data[0].val) {
      fprintf( stderr, "ok.\n" );
    } else {
      fprintf( stderr, "failed!!!\n" );
      fprintf( stderr, "HERE: "  );
      bcopy( &check[data.data[0].nr], bytes, 8 );
      for (i=0;i<8;i++) {
	fprintf( stderr, "[0x%s%x]", bytes[i]<16?"0":"", bytes[i] );
      }
      fprintf( stderr, "\n"  );
      fprintf( stderr, "IPC: "  );
      bcopy( &data.data[0].val, bytes, 8 );
      for (i=0;i<8;i++) {
	fprintf( stderr, "[0x%s%x]", bytes[i]<16?"0":"", bytes[i] );
      }
      fprintf( stderr, "\n"  );
    }
    
  }

}

/**************************************************************************/
/*   TEST 4                                                               */
/**************************************************************************/

static void 
ipc_test_4_handler( MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		    void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE                      err = IPC_OK;
  FORMATTER_PTR                        formatter;
  unsigned char                        bytes[8];
  int                                  i;
  static carmen_test_4_message_type    data;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &data,
			   sizeof(carmen_test_4_message_type));
  IPC_freeByteArray(callData);
  if (err != IPC_OK) {
    fprintf(stderr, "Could not unmarshall %s\n", 
	    IPC_msgInstanceName(msgRef));
    return;
  }

  if (data.nr>=NUM_CHECKS) {
    fprintf( stderr, "ERROR: idx number (%d) too big!",
	     data.nr );
  } else {
    fprintf( stderr, "CHECK (t:%d/n:%d): %4.8f  .... : ",
	     4, data.nr, check[data.nr] );
    if (check[data.nr] == data.val) {
      fprintf( stderr, "ok.\n" );
    } else {
      fprintf( stderr, "failed!!!\n" );
      fprintf( stderr, "HERE: "  );
      bcopy( &check[data.nr], bytes, 8 );
      for (i=0;i<8;i++) {
	fprintf( stderr, "[0x%s%x]", bytes[i]<16?"0":"", bytes[i] );
      }
      fprintf( stderr, "\n"  );
      fprintf( stderr, "IPC: "  );
      bcopy( &data.val, bytes, 8 );
      for (i=0;i<8;i++) {
	fprintf( stderr, "[0x%s%x]", bytes[i]<16?"0":"", bytes[i] );
      }
      fprintf( stderr, "\n"  );
    }
    
  }

}

/**************************************************************************/
/*   TEST 5                                                               */
/**************************************************************************/

static void 
ipc_test_5_handler( MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		    void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE                      err = IPC_OK;
  FORMATTER_PTR                        formatter;
  unsigned char                        bytes[8];
  int                                  i;
  static carmen_test_5_message_type    data;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &data,
			   sizeof(carmen_test_5_message_type));
  IPC_freeByteArray(callData);
  if (err != IPC_OK) {
    fprintf(stderr, "Could not unmarshall %s\n", 
	    IPC_msgInstanceName(msgRef));
    return;
  }

  if (data.data[0].nr>=NUM_CHECKS) {
    fprintf( stderr, "ERROR: idx number (%d) too big!",
	     data.data[0].nr );
  } else {
    fprintf( stderr, "CHECK (t:%d/n:%d): %4.8f  .... : ",
	     5, data.data[0].nr, check[data.data[0].nr] );
    if (check[data.data[0].nr] == data.data[0].val) {
      fprintf( stderr, "ok.\n" );
    } else {
      fprintf( stderr, "failed!!!\n" );
      fprintf( stderr, "HERE: "  );
      bcopy( &check[data.data[0].nr], bytes, 8 );
      for (i=0;i<8;i++) {
	fprintf( stderr, "[0x%s%x]", bytes[i]<16?"0":"", bytes[i] );
      }
      fprintf( stderr, "\n"  );
      fprintf( stderr, "IPC: "  );
      bcopy( &data.data[0].val, bytes, 8 );
      for (i=0;i<8;i++) {
	fprintf( stderr, "[0x%s%x]", bytes[i]<16?"0":"", bytes[i] );
      }
      fprintf( stderr, "\n"  );
    }
    
  }

}

void
ipc_send_test( int type, int nr, double val )
{
  IPC_RETURN_TYPE               err;
  carmen_test_1_message_type    test1;
  carmen_test_2_message_type    test2;
  carmen_test_3_message_type    test3;
  carmen_test_4_message_type    test4;
  carmen_test_5_message_type    test5;

  switch(type) {
  case 1:
    test1.nr    = nr;
    test1.val   = val;
    err = IPC_publishData (CARMEN_TEST_1_MESSAGE_NAME, &test1 );
    if (err != IPC_OK)
      fprintf(stderr, "Could not publish %s\n", CARMEN_TEST_1_MESSAGE_NAME);
    break;
  case 2:
    test2.data.nr    = nr;
    test2.data.val   = val;
    err = IPC_publishData (CARMEN_TEST_2_MESSAGE_NAME, &test2 );
    if (err != IPC_OK)
      fprintf(stderr, "Could not publish %s\n", CARMEN_TEST_2_MESSAGE_NAME);
    break;
  case 3:
    test3.nr = 1;
    test3.data  =
      (carmen_test_1_message_type *) malloc( test3.nr * sizeof(carmen_test_1_message_type) ); /* check_alloc checked */
    test3.data[0].nr    = nr;
    test3.data[0].val   = val;
    err = IPC_publishData (CARMEN_TEST_3_MESSAGE_NAME, &test3 );
    if (err != IPC_OK)
      fprintf(stderr, "Could not publish %s\n", CARMEN_TEST_3_MESSAGE_NAME);
    break;
  case 4:
    test4.len   = 0;
    test4.nr    = nr;
    test4.val   = val;
    err = IPC_publishData (CARMEN_TEST_4_MESSAGE_NAME, &test4 );
    if (err != IPC_OK)
      fprintf(stderr, "Could not publish %s\n", CARMEN_TEST_4_MESSAGE_NAME);
    break;
  case 5:
    test5.data[0].nr    = nr;
    test5.data[0].val   = val;
    err = IPC_publishData (CARMEN_TEST_5_MESSAGE_NAME, &test5 );
    if (err != IPC_OK)
      fprintf(stderr, "Could not publish %s\n", CARMEN_TEST_5_MESSAGE_NAME);
    break;
  default:
    break;
  }
}

void
ipc_initialize_messages( int sender )
{
  IPC_RETURN_TYPE err = IPC_OK;

  if (!sender) {
    
    err = IPC_defineMsg(CARMEN_TEST_1_MESSAGE_NAME, IPC_VARIABLE_LENGTH, 
			CARMEN_TEST_1_MESSAGE_FMT);
    if (err != IPC_OK) {
      fprintf(stderr, "Could not define %s", CARMEN_TEST_1_MESSAGE_NAME);
      exit(-1);
    }
    
    err = IPC_subscribe(CARMEN_TEST_1_MESSAGE_NAME, ipc_test_1_handler, NULL);
    IPC_setMsgQueueLength(CARMEN_TEST_1_MESSAGE_NAME, 30 );
    if (err != IPC_OK) {
      fprintf(stderr, "Could not subscribe %s", CARMEN_TEST_1_MESSAGE_NAME);
      exit(-1);
    }

    err = IPC_defineMsg(CARMEN_TEST_2_MESSAGE_NAME, IPC_VARIABLE_LENGTH, 
			CARMEN_TEST_2_MESSAGE_FMT);
    if (err != IPC_OK) {
      fprintf(stderr, "Could not define %s", CARMEN_TEST_2_MESSAGE_NAME);
      exit(-1);
    }
    
    err = IPC_subscribe(CARMEN_TEST_2_MESSAGE_NAME, ipc_test_2_handler, NULL);
    IPC_setMsgQueueLength(CARMEN_TEST_2_MESSAGE_NAME, 30 );
    if (err != IPC_OK) {
      fprintf(stderr, "Could not subscribe %s", CARMEN_TEST_2_MESSAGE_NAME);
      exit(-1);
    }

    err = IPC_defineMsg(CARMEN_TEST_3_MESSAGE_NAME, IPC_VARIABLE_LENGTH, 
			CARMEN_TEST_3_MESSAGE_FMT);
    if (err != IPC_OK) {
      fprintf(stderr, "Could not define %s", CARMEN_TEST_3_MESSAGE_NAME);
      exit(-1);
    }
    
    err = IPC_subscribe(CARMEN_TEST_3_MESSAGE_NAME, ipc_test_3_handler, NULL);
    IPC_setMsgQueueLength(CARMEN_TEST_3_MESSAGE_NAME, 30 );
    if (err != IPC_OK) {
      fprintf(stderr, "Could not subscribe %s", CARMEN_TEST_3_MESSAGE_NAME);
      exit(-1);
    }

    err = IPC_defineMsg(CARMEN_TEST_4_MESSAGE_NAME, IPC_VARIABLE_LENGTH, 
			CARMEN_TEST_4_MESSAGE_FMT);
    if (err != IPC_OK) {
      fprintf(stderr, "Could not define %s", CARMEN_TEST_4_MESSAGE_NAME);
      exit(-1);
    }
    
    err = IPC_subscribe(CARMEN_TEST_4_MESSAGE_NAME, ipc_test_4_handler, NULL);
    IPC_setMsgQueueLength(CARMEN_TEST_4_MESSAGE_NAME, 30 );
    if (err != IPC_OK) {
      fprintf(stderr, "Could not subscribe %s", CARMEN_TEST_4_MESSAGE_NAME);
      exit(-1);
    }

    err = IPC_defineMsg(CARMEN_TEST_5_MESSAGE_NAME, IPC_VARIABLE_LENGTH, 
			CARMEN_TEST_5_MESSAGE_FMT);
    if (err != IPC_OK) {
      fprintf(stderr, "Could not define %s", CARMEN_TEST_5_MESSAGE_NAME);
      exit(-1);
    }
    
    err = IPC_subscribe(CARMEN_TEST_5_MESSAGE_NAME, ipc_test_5_handler, NULL);
    IPC_setMsgQueueLength(CARMEN_TEST_5_MESSAGE_NAME, 30 );
    if (err != IPC_OK) {
      fprintf(stderr, "Could not subscribe %s", CARMEN_TEST_5_MESSAGE_NAME);
      exit(-1);
    }

  }
}

void
ipc_update( void )
{
  IPC_listen(0);
}

void
ipc_init( char * name, int sender )
{
  IPC_RETURN_TYPE err;
  char ipc_name[200];

  snprintf(ipc_name, 200, "%s-%d", name, getpid());

  /* set verbosity level */
  IPC_setVerbosity(IPC_Silent);

  /* connect to the central server */
  err = IPC_connect(ipc_name);
  if(err == IPC_Error) {
    fprintf(stderr, "Could not connect to central.\n\n");
    fprintf(stderr, "Did you remember to start central?\n");
    fprintf(stderr, "Did you remember to set your CENTRALHOST variable?");
    fprintf(stderr, " It is currently ");
    if (getenv("CENTRALHOST"))
      fprintf(stderr, "\nset to %s.\n", getenv("CENTRALHOST"));
    else
      fprintf(stderr, "not set.\n");
    exit(-1);
  }
  /* Set local message queue capacity */
  err = IPC_setCapacity(4);
  if (err != IPC_OK) {
    fprintf(stderr, "I had problems setting the IPC capacity. This is a "
	    "very strange error and should never happen.\n");
    exit(-1);
  }
  
  ipc_initialize_messages( sender );
}
  
void
ipc_stop( void )
{
  fprintf( stderr, "INFO: close connection to CENTRAL\n" );
  IPC_disconnect();
}

void
print_usage( void )
{
  fprintf( stderr, "usage: ipc-endian-test [-server]\n" );
}

int
main( int argc, char *argv[] )
{
  int i, t, sender = FALSE;

  if (argc == 2 && !strcmp(argv[1],"-sender")) {
    sender = TRUE;
  } else if (argc != 1) {
    print_usage();
    exit(1);
  }

  ipc_init( argv[0], sender );
  
  if (sender) { 
    for (t=1;t<=5;t++) {
      for (i=0;i<NUM_CHECKS;i++) {
	ipc_send_test( t, i, check[i] );
      }
    }
  } else {
    while(1) {
      ipc_update();
      usleep(10000);
    }
  }

  return(0);
  
}

