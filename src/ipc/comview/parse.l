%{

#define BUFLEN (MAX_LONG_MESSAGE)
#define YY_USE_PROTO

void setInputBuffer(char *inputBuffer); // Forward reference

static int bindex;
static char buf[BUFLEN];

void setInputBuffer(char *inputBuffer)
{ 
  register int inlen = strlen(inputBuffer);

  if (inlen > BUFLEN-1) {
    fprintf(stderr, "LINE TOO LONG -- TRUNCATING (%s)\n", inputBuffer);
    inlen = BUFLEN-1;
  }
  strncpy(buf, inputBuffer, inlen+1); /* Copy the null terminator, as well */

  /* Add a trailing newline, if there is none */
  if (buf[inlen-1] != '\n') {
    buf[inlen] = '\n'; buf[inlen+1] = '\0';
  }
  bindex = 0;
}

/*#undef input*/
/*#define input() (buf[bindex]=='\0' ? 0 : (bindex++, buf[bindex-1]))*/
/*#undef unput*/
/*#define unput(c) if (bindex>0) { bindex--; buf[bindex] = (c); }*/
#undef YY_INPUT
#define YY_INPUT(buffer,result,max_size)\
{\
  if (buf[bindex]== '\0') {\
    result = YY_NULL;\
    buffer[0] = '\0';\
  } else {\
    result = 1;\
    buffer[0] = buf[bindex++];\
  }\
}

%}
NAME_START	[a-zA-Z_\*0-9]
NAME_MID	[a-zA-Z\-_\*0-9]
NAME_END	[a-zA-Z\*0-9]
SEQACH		"SEQUENTIAL ACHIEVEMENT"
SEQPLN		"SEQUENTIAL PLANNING"
DLYPLN		"DELAY PLANNING"
PLNFRST		"PLAN FIRST"
BETWEEN		" constraint between"
TO		" constraint to"
%%
^"Query"			{ return QUERY_TOKEN; }
^"Goal"				{ return GOAL_TOKEN; }
^"Command"			{ return COMMAND_TOKEN; }
^"Inform"			{ return INFORM_TOKEN; }
^"Constraint"			{ return INFORM_TOKEN; }
^"Broadcast"			{ return BROADCAST_TOKEN; }
^"Exception"			{ return EXCEPTION_TOKEN; }
^"Killed"			{ return KILLED_TOKEN; }
^"Will kill"			{ return KILL_WHEN_TOKEN; }
^"IMonitor"			{ return INT_MON_TOKEN; }
^"Monitor"			{ return INT_MON_TOKEN; }
^"PMonitor"			{ return PT_MON_TOKEN; }
^"FireDemon"			{ return FIRE_DEMON_TOKEN; }
^" Received a new connection"	{ return RECEIVED_A_NEW_CONNECTION; }
^"  Retry"			{ return RETRY_TOKEN; }
^"  Reply"			{ return REPLY_TOKEN; }
^"  Failure"			{ return FAILURE_TOKEN; }
^"  Success"			{ return SUCCESS_TOKEN; }
^"  Done" 			{ return SUCCESS_TOKEN; }
^"  ByPass"			{ return BYPASS_TOKEN; }
^"   Asserting"			{ return PT_CONSTR_TOKEN; }
^"   modName"			{ return MODNAME_TOKEN; }
^"   hostName"			{ return HOSTNAME_TOKEN; }
^"    ".+			{ return DATA_TOKEN; }
^" Closing"			{ return CLOSING_TOKEN; }
^"Adding"			{ return TASK_CONSTR_TOKEN; }
"-->"				{ return ARROW_TOKEN; }
"and"				{ return AND_TOKEN; }
"on"				{ return ON_TOKEN; }
"PENDING LIMIT: Deleted message"  { return PENDING_LIMIT_TOKEN; }
"(Sent)"			{ yylval.ival = SENT;
				  return STATUS_TOKEN; }
"(Pending)"			{ yylval.ival = PENDING;
				  return STATUS_TOKEN; }
"(Inactive)"			{ yylval.ival = INACTIVE;
				  return STATUS_TOKEN; }
"(Completed)"			{ yylval.ival = COMPLETED;
				  return STATUS_TOKEN; }
"Start("			{ yylval.ival = START;
				  return POINT_TOKEN; }
"End("				{ yylval.ival = END;
				  return POINT_TOKEN; }
"Planning("			{ yylval.ival = PLANNING_INTERVAL;
				  return INTERVAL_TOKEN; }
"Achievement("			{ yylval.ival = ACHIVEMENT_INTERVAL;
				  return INTERVAL_TOKEN; }
"Handling("			{ yylval.ival = HANDLING_INTERVAL;
				  return INTERVAL_TOKEN; }
[=<>]				{ yylval.cval = yytext[0];
				  return RELP_TOKEN; }
">="				{ yylval.cval = '/';
				  return RELP_TOKEN; }
"<="				{ yylval.cval = '\\';
				  return RELP_TOKEN; }
{SEQACH}{BETWEEN}		{ yylval.ival = MSG_SEQ_ACH;
				  return PREDEF_CONSTR_TOKEN; }
{SEQPLN}{BETWEEN}		{ yylval.ival = MSG_SEQ_PLAN;
				  return PREDEF_CONSTR_TOKEN; }
{DLYPLN}{TO}			{ yylval.ival = MSG_DELAY_PLAN;
				  return PREDEF_CONSTR_TOKEN; }
{PLNFRST}{TO}			{ yylval.ival = MSG_PLAN_FIRST;
				  return PREDEF_CONSTR_TOKEN; }
"when it completes"		{ yylval.ival = WHEN_COMPLETES;
				  return SITUATION_TOKEN; }
"when submessages complete"	{ yylval.ival = WHEN_SUBMESSAGES_COMPLETES;
				  return SITUATION_TOKEN; }
"when all references to it are released" { yylval.ival = WHEN_REFERENCES_RELEASED;
				      return SITUATION_TOKEN; }
[0-9]+				{ yylval.ival = atoi(yytext);
				  return INTEGER_TOKEN; }
"-1"				{ yylval.ival = -1;
  				  return INTEGER_TOKEN; }
"Resource"			{  } /* Ignore this word */
{NAME_START}{NAME_MID}*{NAME_END}? { yylval.string = strdup((char *)yytext);
				    return NAME_TOKEN; }
[\.{}():\n]			{ return yytext[0]; } /* Single char tokens */
[\ \t]+				; /* ignore except to delimit tokens */
[\'\,\"\?\[\]\!\*\%\/\-a]	; /* extra characters in data lines */
%%
