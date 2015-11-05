%{
#include "Standard.h"
#include "List.h"
#include "Array.h"
#include "MsgData.h"
#include "TaskTree.h"
#include "Parser.h"

void yyerror(char *s); /* Forward reference */
int yywrap(void); /* Forward reference */
int yylex(void); /* Forward reference */

#define YYDEBUG 1

static void transferTime(MsgData from, MsgData to)
{
  to->time = from->time;
  to->hours = from->hours; 
  to->minutes = from->minutes;
  to->seconds = from->seconds;
  to->m_seconds = from->m_seconds; 
}

static void transferMsgRef(MsgData from, MsgData to)
{
  to->name = from->name;
  to->id = from->id;
}

%}
%start start
%union {
  char  cval;
  int   ival;
  char *string;
  MsgData msgData; /* Pointer to the returned value of the parse */
  _MsgData temp;   /* A temporary MsgData struct (not a pointer) */
}
%token QUERY_TOKEN
%token GOAL_TOKEN
%token COMMAND_TOKEN
%token INFORM_TOKEN
%token BROADCAST_TOKEN
%token EXCEPTION_TOKEN
%token KILLED_TOKEN
%token KILL_WHEN_TOKEN
%token INT_MON_TOKEN
%token PT_MON_TOKEN
%token FIRE_DEMON_TOKEN
%token RECEIVED_A_NEW_CONNECTION
%token DATA_TOKEN
%token RETRY_TOKEN
%token REPLY_TOKEN
%token FAILURE_TOKEN
%token SUCCESS_TOKEN
%token BYPASS_TOKEN
%token MODNAME_TOKEN
%token CLOSING_TOKEN
%token HOSTNAME_TOKEN
%token PT_CONSTR_TOKEN
%token TASK_CONSTR_TOKEN
%token ARROW_TOKEN
%token AND_TOKEN
%token ON_TOKEN
%token PENDING_LIMIT_TOKEN
%token <ival> STATUS_TOKEN
%token <ival> POINT_TOKEN
%token <ival> INTERVAL_TOKEN
%token <cval> RELP_TOKEN
%token <ival> PREDEF_CONSTR_TOKEN
%token <ival> SITUATION_TOKEN
%token <ival> INTEGER_TOKEN
%token <string> NAME_TOKEN
%type <msgData> start statement message response constraint termination
%type <msgData> query goal inform broadcast command interval_monitor
%type <msgData> point_monitor exception
%type <msgData> reply failure success retry bypass fire_demon
%type <msgData> point_constraint task_constraint killed kill_when
%type <msgData> pending_limit
%type <msgData> module_status connect disconnect host received_a_new_connection received_data
%type <msgData> msg_body msg_ref_c task_constraint1 
%type <string>  multi_word msg_name module_name host_name
%type <ival>    id
%type <temp>    msg_ref time time_point
%%
start :
	statement '\n' { parsedMessage = $$;}
	| error '\n' { parsedMessage = $$ = CreateMsgData();
		       $$->type = TYPE_NULL; }
	;

statement :
	message
	| response
	| constraint
	| termination
	| module_status
	;

message :
	query
	| goal
	| inform
	| broadcast
	| command
	| interval_monitor
	| point_monitor
	| exception
	;

response :
	reply
	| failure
	| success
	| retry
	| bypass
	| fire_demon
	;

constraint :
	point_constraint
	| task_constraint
	;

termination :
	killed
	| kill_when  
	| pending_limit
	;

module_status :
	connect
	| disconnect
        | host
	| received_a_new_connection
	| received_data
	;

query :
	QUERY_TOKEN msg_body { $$ = $2; $$->type = QUERY; }
	;

goal :
	GOAL_TOKEN msg_body { $$ = $2; $$->type = GOAL; }
	;

inform :
	INFORM_TOKEN msg_body { $$ = $2; $$->type = INFORM; }
	;

broadcast :
	BROADCAST_TOKEN msg_body { $$ = $2; $$->type = BROADCAST; }
	;

command :
	COMMAND_TOKEN msg_body { $$ = $2; $$->type = COMMAND; }
	;

interval_monitor :
	INT_MON_TOKEN msg_body { $$ = $2; $$->type = INTERVAL_MONITOR; }
	;

point_monitor :
	PT_MON_TOKEN msg_body { $$ = $2; $$->type = POINT_MONITOR; }
	;

exception :
	EXCEPTION_TOKEN msg_body { $$ = $2; $$->type = EXCEPTION; }
	;

reply :
	REPLY_TOKEN msg_ref_c module_name ARROW_TOKEN module_name id time
	{ $$ = $2; $$->type = REPLY; $$->source = $3; $$->parent_id = $6;
	  $$->dest = $5; transferTime(&$7, $$); }
	;

failure :
	FAILURE_TOKEN msg_ref_c module_name id time 
	 { $$ = $2; $$->type = FAILURE; 
	   $$->source = $3; $$->parent_id = $4; transferTime(&$5, $$); }
	;

success :
	SUCCESS_TOKEN msg_ref_c time
	 { $$ = $2; $$->type = SUCCESS; transferTime(&$3, $$); }
	;

retry :
	RETRY_TOKEN msg_ref
	 { $$ = CreateMsgData(); $$->type = RETRY; transferMsgRef(&$2, $$); }
	;

bypass :
	BYPASS_TOKEN msg_body { $$ = $2; $$->type = BYPASS; }
	;

fire_demon :
	FIRE_DEMON_TOKEN id ':' module_name id ARROW_TOKEN module_name 
	STATUS_TOKEN time
	{ $$ = CreateMsgData(); $$->type = FIRE_DEMON;
	  $$->id = $2; $$->source = $4; $$->parent_id = $5;
	  $$->dest = $7; $$->status = $8; transferTime(&$9, $$); }
	;

killed :
	KILLED_TOKEN msg_ref time
	 { $$ = CreateMsgData(); $$->type = KILLED; 
	   transferMsgRef(&$2, $$); transferTime(&$3, $$); }
	;

pending_limit :
	PENDING_LIMIT_TOKEN msg_ref time
	 { $$ = CreateMsgData(); $$->type = KILLED; 
	   transferMsgRef(&$2, $$); transferTime(&$3, $$); }
	;

kill_when :
	KILL_WHEN_TOKEN msg_ref SITUATION_TOKEN time
	 { $$ = CreateMsgData(); $$->type = WILL_KILL;
	   transferMsgRef(&$2, $$); $$->kill_when = $3; }
	;

connect :
	MODNAME_TOKEN ':' multi_word
	{ $$ = CreateMsgData(); $$->type = MODULE_CONNECT;
	  $$->name = $3; }
	;

disconnect :
	CLOSING_TOKEN multi_word ON_TOKEN host_name
	{ $$ = CreateMsgData(); $$->type = MODULE_DISCONNECT;
	  $$->name = $2; $$->source = $4; }
	;

host :
	HOSTNAME_TOKEN ':' host_name
	{ $$ = CreateMsgData(); $$->type = HOSTNAME;
	  $$->name = $3; }
	;

received_a_new_connection :
	RECEIVED_A_NEW_CONNECTION ':' INTEGER_TOKEN
	{ $$ = CreateMsgData(); $$->type = UNUSED_INFO; }
	;

received_data :
	DATA_TOKEN
	{ $$ = CreateMsgData(); $$->type = UNUSED_INFO; }
	;

multi_word :
	NAME_TOKEN
	| multi_word NAME_TOKEN
	  { register int len = 2+strlen($1)+strlen($2);
	    $$ = realloc($1, len); 
	    strcat($$, " "); strcat($$, $2); $$[len-1] = '\0'; free($2); }
	;

host_name :
	NAME_TOKEN
	| host_name '.' NAME_TOKEN
	  { register int len = 2+strlen($1)+strlen($3);
	    $$ = realloc($1, len); 
	    strcat($$, "."); strcat($$, $3); $$[len-1] = '\0'; free($3); }
	;

msg_name :
	multi_word
	;

module_name :
	multi_word
	;

id :
	/* Nothing - optional */ { $$ = -1; /* Nothing */ }
	| '{' INTEGER_TOKEN '}'  { $$ = (($2 == -1) ? ROOT_NODE_ID : $2); }
	;

time :
	/* Nothing - optional */ 
     	  { $$.time = $$.hours = $$.minutes = $$.seconds = $$.m_seconds = 0; }
	| INTEGER_TOKEN ':' INTEGER_TOKEN ':' INTEGER_TOKEN '.' INTEGER_TOKEN
	  { $$.hours = $1; $$.minutes = $3;
            $$.seconds = $5; $$.m_seconds = 10*$7;
	    $$.time = $$.m_seconds + 1000*($$.seconds + 
					   60*($$.minutes + 60*$$.hours)); }
	;

msg_ref :
	msg_name id { $$.name = $1; $$.id = $2; }
	;

msg_ref_c : 
	msg_ref ':'
	 { $$ = CreateMsgData(); transferMsgRef(&$1, $$); }
	;

msg_body :
	msg_ref_c module_name id ARROW_TOKEN module_name STATUS_TOKEN time
	 { $$ = $1; $$->source = $2; $$->parent_id = $3; $$->dest = $5;
	   $$->status = $6; transferTime(&$7, $$); }
        | msg_ref_c STATUS_TOKEN time
	 { $$ = $1; $$->status = $2; transferTime(&$3, $$); }

	;

time_point :
	POINT_TOKEN INTERVAL_TOKEN msg_name ')' ')' id
	{ $$.point1 = $1; $$.interval1 = $2; $$.msg1 = $3; $$.id1 = $6; }
	;

point_constraint :
	PT_CONSTR_TOKEN time_point RELP_TOKEN time_point
	{ $$ = CreateMsgData(); $$->type = POINT_CONSTRAINT;
	  $$->point1 = $2.point1; $$->interval1 = $2.interval1;
	  $$->msg1 = $2.msg1; $$->id1 = $2.id1; $$->point_TC = $3;
	  $$->point2 = $4.point2; $$->interval2 = $4.interval2;
	  $$->msg2 = $4.msg2; $$->id2 = $2.id2; }
	;

task_constraint1 :
	TASK_CONSTR_TOKEN PREDEF_CONSTR_TOKEN msg_ref
	 { $$ = CreateMsgData(); $$->type = TEMP_CONSTRAINT;
	   $$->temporal_constraint = $2; $$->msg1 = $3.name; $$->id1 = $3.id; }
	;

task_constraint :
	task_constraint1
	| task_constraint1 AND_TOKEN msg_ref
	  { $$ = $1; $$->msg2 = $3.name; $$->id2 = $3.id; }
	;
%%
/* static int yylook(void);*/ /* Forward reference */
/* static int yyback(int *p, int m);*/ /* Forward reference */

#include "./lex.yy.c"

int yywrap (void)
{
  return 1;
}

void yyerror(char *s)
{
  if (strcmp(s, "parse error"))
    fprintf(stderr, "%s: %s (at %s)\n",
	    __FILE__, s, yytext);
}
