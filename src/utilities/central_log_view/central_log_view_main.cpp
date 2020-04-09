#include <carmen/carmen.h>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

FILE   *g_logfile = NULL;
char   *g_logfile_name = NULL;
char    g_logstarttime[25] = {0};
char    g_logtime[13] = {0};
char    g_logtime_first[13] = {0};
char    g_logtime_last[13] = {0};
long    g_avgSize = -1;
long    g_avgSize_last = -1;
float   g_msgPer;
float   g_msgPer_last;
float   g_waitPer;
float   g_waitPer_last;
bool    g_filter_time_all = true;
double  g_filter_time_initial = 0.0;
double  g_filter_time_final = (24.0 * 60.0 * 60.0 - 0.001);
bool    g_filter_class_all = true;
bool    g_filter_class[] = {true, true, true, true, true, true, true, true, true, true, true, true, true};
bool    g_filter_msg_all = true;
bool    g_filter_mod_all = true;
bool    g_filter_keyword_all = true;
vector<string> g_filter_msg_vec;
vector<string> g_filter_orig_vec;
vector<string> g_filter_dest_vec;
vector<string> g_filter_key_vec;
vector<string> g_filter_inv_msg_vec;
vector<string> g_filter_inv_orig_vec;
vector<string> g_filter_inv_dest_vec;
vector<string> g_filter_inv_key_vec;
bool    g_summary_connection = false;
bool    g_summary_registration = false;
bool    g_summary_orig_message = false;
bool    g_summary_dest_message = false;
int     g_verbose = 0;
bool    g_tabular = false;
bool    g_print_usage = false;


bool
wildcard_strcmp(const char *wildcard, const char *str)
{
	const char *cp = NULL, *mp = NULL;

	while ((*str != 0) && (*wildcard != '*'))
	{
		if ((*wildcard != *str) && (*wildcard != '?'))
			return (false);

		wildcard++;
		str++;
	}

	while (*str != 0)
	{
		if (*wildcard == '*')
		{
		  wildcard++;
		  if (*wildcard == 0)
			return (true);

		  mp = wildcard;
		  cp = str + 1;
		}
		else if ((*wildcard == *str) || (*wildcard == '?'))
		{
		  wildcard++;
		  str++;
		}
		else
		{
		  wildcard = mp;
		  str = cp;
		  cp++;
		}
	}

	while (*wildcard == '*')
		wildcard++;
	bool cmp = (*wildcard == 0);

	return (cmp); // True if get a match
}


double
time_str2dbl(char *time_str)
{
	int hours = 0, minutes = 0, pos = 0;
	double secs = 0.0;

	if ((sscanf(time_str, "%d:%d:%lf %n", &hours, &minutes, &secs, &pos) == 3) ||
		(sscanf(time_str, "%d:%d %n", &hours, &minutes, &pos) == 2) ||
		(sscanf(time_str, "%d %n", &hours, &pos) == 1))
	{
		if ((time_str[pos] == 0) && (hours >= 0) && (minutes >= 0 && minutes < 60) && (secs >= 0.0 && secs < 60.0))
		{
			double time_dbl = (((hours * 60.0) + minutes) * 60.0) + secs;

			return (time_dbl);
		}
	}
	return (-1.0);
}


double
elapsed_time(char *time_str)
{
	if (g_logtime_first[0] == 0)
		return (0.0);

	double elapsed = (time_str2dbl(time_str) - time_str2dbl(g_logtime_first));

	return (elapsed);
}


bool
filter_time(char *time_str)
{
	if (g_filter_time_all)
		return (true);

	double logtime_secs = time_str2dbl(time_str);
	bool filtered = (g_filter_time_initial <= logtime_secs && g_filter_time_final >= logtime_secs);

	return (filtered);
}


bool
filter_class(int msg_class_num)
{
	if (g_filter_class_all)
		return (true);

	bool filtered = g_filter_class[msg_class_num];

	return (filtered);
}


bool
filter_message(string msg)
{
	if (g_filter_msg_all)
		return (true);

	for (int i = 0; i < (int) g_filter_inv_msg_vec.size(); i++)
	{
		bool filtered = wildcard_strcmp(g_filter_inv_msg_vec[i].c_str(), msg.c_str());
		if (filtered)
			return (false);
	}
	if (g_filter_msg_vec.empty())
		return (true);

	for (int i = 0; i < (int) g_filter_msg_vec.size(); i++)
	{
		bool filtered = wildcard_strcmp(g_filter_msg_vec[i].c_str(), msg.c_str());
		if (filtered)
			return (true);
	}

	return (false);
}


bool
filter_module(string mod)
{
	if (g_filter_mod_all)
		return (true);

	for (int i = 0; i < (int) g_filter_inv_orig_vec.size(); i++)
	{
		bool filtered = (wildcard_strcmp(g_filter_inv_orig_vec[i].c_str(), mod.c_str()) || wildcard_strcmp(g_filter_inv_dest_vec[i].c_str(), mod.c_str()));
		if (filtered)
			return (false);
	}
	if (g_filter_orig_vec.empty())
		return (true);

	for (int i = 0; i < (int) g_filter_orig_vec.size(); i++)
	{
		bool filtered = (wildcard_strcmp(g_filter_orig_vec[i].c_str(), mod.c_str()) || wildcard_strcmp(g_filter_dest_vec[i].c_str(), mod.c_str()));
		if (filtered)
			return (true);
	}

	return (false);
}


bool
filter_module(string orig, string dest)
{
	if (g_filter_mod_all)
		return (true);

	for (int i = 0; i < (int) g_filter_inv_orig_vec.size(); i++)
	{
		bool filtered = (wildcard_strcmp(g_filter_inv_orig_vec[i].c_str(), orig.c_str()) && wildcard_strcmp(g_filter_inv_dest_vec[i].c_str(), dest.c_str()));
		if (filtered)
			return (false);
	}
	if (g_filter_orig_vec.empty())
		return (true);

	for (int i = 0; i < (int) g_filter_orig_vec.size(); i++)
	{
		bool filtered = (wildcard_strcmp(g_filter_orig_vec[i].c_str(), orig.c_str()) && wildcard_strcmp(g_filter_dest_vec[i].c_str(), dest.c_str()));
		if (filtered)
			return (true);
	}

	return (false);
}


bool
filter_keyword(string text)
{
	if (g_filter_keyword_all)
		return (true);

	for (int i = 0; i < (int) g_filter_inv_key_vec.size(); i++)
	{
		string wildcard = '*' + g_filter_inv_key_vec[i] + '*';
		bool filtered = wildcard_strcmp(wildcard.c_str(), text.c_str());
		if (filtered)
			return (false);
	}
	if (g_filter_key_vec.empty())
		return (true);

	for (int i = 0; i < (int) g_filter_key_vec.size(); i++)
	{
		string wildcard = '*' + g_filter_key_vec[i] + '*';
		bool filtered = wildcard_strcmp(wildcard.c_str(), text.c_str());
		if (filtered)
			return (true);
	}

	return (false);
}


enum g_log_record_class {CONNECTION_CLASS, REGISTRATION_CLASS, BROADCAST_CLASS, INFORM_CLASS, DONE_CLASS, QUERY_CLASS, REPLY_CLASS, DELETED_CLASS,
	DATA_CLASS, HANDLE_SUMMARY_CLASS, OTHER_CLASS, UNKNOWN_MESSAGE_CLASS, UNKNOWN_CLASS};

const char *g_log_record_class_name[] = {"Connection", "Registration", "Broadcast", "Inform", "Done", "Query", "Reply", "Deleted",
	"Data", "Handle_Summary", "Other", "Unknown_Message", "Unknown_Class"};

#define LOG_RECORD_CLASS_SIZE 13
int g_log_record_class_count[LOG_RECORD_CLASS_SIZE] = {0};
int g_log_record_class_lines[LOG_RECORD_CLASS_SIZE] = {0};


int
get_log_record_class(char *msg_class)
{
	for (int i = 0; i < LOG_RECORD_CLASS_SIZE; i++)
	{
		if (strcmp(g_log_record_class_name[i], msg_class) == 0)
			return (i);
	}
	return (-1);
}


void
update_logtime()
{
	if (g_logtime[0] != 0)
	{
		strcpy(g_logtime_last, g_logtime);
		if (g_logtime_first[0] == 0)
			strcpy(g_logtime_first, g_logtime);
	}
}


void
update_handle_summary()
{
	if (g_avgSize >= 0)
	{
		g_msgPer_last  = g_msgPer;
		g_waitPer_last = g_waitPer;
		g_avgSize_last = g_avgSize;
	}
}


void
update_logstats(int message_class_num, int lines)
{
	update_logtime();
	update_handle_summary();

	g_log_record_class_count[message_class_num] += 1;
	g_log_record_class_lines[message_class_num] += lines;
}


void
print_general_summary()
{
	int lines_total = 0;

	printf("\n");
	if (g_logtime_last[0] != 0)
		printf("Selected log period:  from %s to %s (%.3lf secs)\n", g_logtime_first, g_logtime_last, elapsed_time(g_logtime_last));
	if (g_avgSize_last >= 0)
		printf("Handle time summary:  Messaging: %.3f%%  Waiting: %.3f%%  Average size: %ld bytes\n", g_msgPer_last, g_waitPer_last, g_avgSize_last);

	printf("\n==========================================\n%s" "\n==========================================\n", "General Summary");
	printf("Log Record Class          Count      Lines\n" "--------------------  ---------  ---------\n");
	for (int i = 0; i < LOG_RECORD_CLASS_SIZE; i++)
	{
		if (g_filter_class_all || g_filter_class[i])
			printf("%-20s  %9d  %9d\n", g_log_record_class_name[i], g_log_record_class_count[i], g_log_record_class_lines[i]);

		lines_total += g_log_record_class_lines[i];
	}
	printf("\n%-31s  %9d" "\n==========================================\n\n", "Totals:", lines_total);
}


struct module_t
{
	string module;
	int connection_count;
	int close_count;
};

vector<module_t> g_module_vec;


int
locate_module(string module)
{
	for (int i = 0; i < (int) g_module_vec.size(); i++)
	{
		if (g_module_vec[i].module == module)
			return (i);
	}
	return (-1);
}


void
update_module_vec(string module, int connection_count, int close_count)
{
	int i = locate_module(module);
	if (i < 0)
	{
		module_t module_rec = {module, 0, 0};
		g_module_vec.push_back(module_rec);
		i = g_module_vec.size() - 1;
	}
	g_module_vec[i].connection_count += connection_count;
	g_module_vec[i].close_count += close_count;
}


bool
module_order_by_name (module_t a, module_t b)
{
	bool order = (a.module < b.module);

	return (order);
}


void
print_module_summary()
{
	int connection_total = 0, close_total = 0;

	printf("\n==============================================================\n%s"
		   "\n==============================================================\n", "Module Connection Summary");
	printf("Module                                      Connect      Close\n"
		   "----------------------------------------  ---------  ---------\n");
	sort(g_module_vec.begin(), g_module_vec.end(), module_order_by_name);
	for (int i = 0; i < (int) g_module_vec.size(); i++)
	{
		printf("%-40s  %9d  %9d\n", g_module_vec[i].module.c_str(), g_module_vec[i].connection_count, g_module_vec[i].close_count);
		connection_total += g_module_vec[i].connection_count;
		close_total += g_module_vec[i].close_count;
	}
	printf("\n%-29s  %9d  %9d  %9d" "\n==============================================================\n\n",
			"Totals:", (int) g_module_vec.size(), connection_total, close_total);
}


struct message_t
{
	string msg_name;
	int registration_count;
};

vector<message_t> g_msg_reg_vec;


int
locate_msg_reg(string msg_name)
{
	for (int i = 0; i < (int) g_msg_reg_vec.size(); i++)
	{
		if (g_msg_reg_vec[i].msg_name == msg_name)
			return (i);
	}
	return (-1);
}


void
update_msg_reg_vec(string msg_name, int registration_count)
{
	int i = locate_msg_reg(msg_name);
	if (i < 0)
	{
		message_t msg_reg = {msg_name, 0};
		g_msg_reg_vec.push_back(msg_reg);
		i = g_msg_reg_vec.size() - 1;
	}
	g_msg_reg_vec[i].registration_count += registration_count;
}


bool
msg_reg_order_by_name (message_t a, message_t b)
{
	bool order = (a.msg_name < b.msg_name);

	return (order);
}


void
print_registration_summary()
{
	int registration_total = 0;

	printf("\n================================================================================\n%s"
		   "\n================================================================================\n", "Message Registration Summary");
	printf("Message                                                             Registration\n"
		   "------------------------------------------------------------------  ------------\n");
	sort(g_msg_reg_vec.begin(), g_msg_reg_vec.end(), msg_reg_order_by_name);
	for (int i = 0; i < (int) g_msg_reg_vec.size(); i++)
	{
		printf("%-66s  %12d\n", g_msg_reg_vec[i].msg_name.c_str(), g_msg_reg_vec[i].registration_count);
		registration_total += g_msg_reg_vec[i].registration_count;
	}
	printf("\n%-55s  %9d  %12d" "\n================================================================================\n\n",
			"Totals:", (int) g_msg_reg_vec.size(), registration_total);
}


bool
read_line(FILE *textfile, char *line_c, int *buffersize)
{
	line_c[0] = 0;
	int linesize = 0;
	long textstart = ftell(textfile);
	char c = fgetc(textfile);

	while (!feof(textfile) &&  c != '\n')
	{
		linesize++;
		c = fgetc(textfile);
	}
	if (linesize == 0)
		return (false);

	if (linesize > *buffersize)
	{
		*buffersize = linesize;
		realloc(line_c, linesize + 1);
	}

	fseek(textfile, textstart, SEEK_SET);
	fgets(line_c, linesize + 1, textfile);
	fgetc(textfile); // discard the '\n'

	return (true);
}


bool
is_time_summary_line(char *line_c)
{
	long msg_num;
	char rest[2000];
	bool is_time_summary = (sscanf(line_c, " %ld: Msg: %s", &msg_num, rest) == 2);

	return (is_time_summary);
}


int
get_data(char *line_c, long *linecount, int *buffersize, string &line_data, string &unsplit_data)
{
	char single_line_data[2000];
	int lines = 0;
	long textstart = ftell(g_logfile);

	while (read_line(g_logfile, line_c, buffersize))
	{
		bool isdataline = (strncmp(line_c, "     ", 5) == 0);
		if (lines == 0)
			isdataline = (strncmp(line_c, "    ", 4) == 0 && !is_time_summary_line(line_c));
		if (!isdataline)
		{
			fseek(g_logfile, textstart, SEEK_SET);
			break;
		}
		lines++;
		*linecount += 1;
		textstart = ftell(g_logfile);
		sprintf(single_line_data, "Logline: %9ld  Class: %-10s  Logtime: %s  Text: %s\n", *linecount,
				g_log_record_class_name[DATA_CLASS], g_logtime, line_c);
		line_data += single_line_data;
		unsplit_data += (lines == 1) ? &line_c[4] : &line_c[5];
	}
	return (lines);
}


struct ipc_t
{
	string msg_name;
	string orig_module;
	string dest_module;
	int sent_count;
	int deleted_count;
	int pending_count;
	int on_hold_count;
	int pending_sent_count;
	int on_hold_sent_count;
	int on_hold_pending_count;
};

vector<ipc_t> g_msg_vec;


int
locate_message(string msg_name, string orig_module, string dest_module)
{
	for (int i = 0; i < (int) g_msg_vec.size(); i++)
	{
		if (g_msg_vec[i].msg_name == msg_name && g_msg_vec[i].orig_module == orig_module && g_msg_vec[i].dest_module == dest_module)
			return (i);
	}
	return (-1);
}


bool
msg_order_by_orig (ipc_t a, ipc_t b)
{
	bool order = (a.msg_name < b.msg_name);
	if (a.msg_name == b.msg_name)
	{
		order = (a.orig_module < b.orig_module);
		if (a.orig_module == b.orig_module)
			order = (a.dest_module < b.dest_module);
	}

	return (order);
}


bool
msg_order_by_dest (ipc_t a, ipc_t b)
{
	bool order = (a.msg_name < b.msg_name);
	if (a.msg_name == b.msg_name)
	{
		order = (a.dest_module < b.dest_module);
		if (a.dest_module == b.dest_module)
			order = (a.orig_module < b.orig_module);
	}

	return (order);
}


struct ipc_pend_t
{
	string msg_name;
	int loc_id;
	string orig_module;
	string dest_module;
	string status;
};

vector<ipc_pend_t> g_pending_vec;


int
locate_pending(string msg_name, int loc_id)
{
	for (int i = 0; i < (int) g_pending_vec.size(); i++)
	{
		if (g_pending_vec[i].msg_name == msg_name && g_pending_vec[i].loc_id == loc_id)
			return (i);
	}
	return (-1);
}


void
update_pending_vec(string msg_name, int loc_id, string &orig_module, string &dest_module, string &status)
{
	int i = locate_pending(msg_name, loc_id);

	if (i < 0)
	{
		ipc_pend_t pending_message = {msg_name, loc_id, orig_module, dest_module, status};
		g_pending_vec.push_back(pending_message);
	}
	else
	{
		if (orig_module != "")
		{
			g_pending_vec[i].orig_module = orig_module;
			g_pending_vec[i].dest_module = dest_module;
		}
		else
			orig_module = g_pending_vec[i].orig_module;

		if (dest_module != "")
			g_pending_vec[i].dest_module = dest_module;

		if (status != "")
			g_pending_vec[i].status = status.substr(status.find_last_of('>') + 1); // "pending-->sent" becomes "sent"
	}
}


int
get_pending_vec(string msg_name, int loc_id, string &orig_module, string &dest_module, string &status)
{
	int i = locate_pending(msg_name, loc_id);
	if (i >= 0)
	{
		orig_module = g_pending_vec[i].orig_module;
		dest_module = g_pending_vec[i].dest_module;
		status = g_pending_vec[i].status;
	}
	else
	{
		orig_module = "???";
		dest_module = "???";
		status = "???";
	}
	return (i);
}


void
delete_pending_vec(string msg_name, int loc_id, string &orig_module, string &dest_module, string &status)
{
	int i = get_pending_vec(msg_name, loc_id, orig_module, dest_module, status);
	if (i >= 0)
		g_pending_vec.erase(g_pending_vec.begin() + i);
}


void
update_message_vec(string msg_name, string orig_module, string dest_module, int sent_count, int pending_count, int on_hold_count)
{
	int i = locate_message(msg_name, orig_module, dest_module);
	if (i < 0)
	{
		ipc_t message = {msg_name, orig_module, dest_module, 0, 0, 0, 0, 0, 0, 0};
		g_msg_vec.push_back(message);
		i = g_msg_vec.size() - 1;
	}
	g_msg_vec[i].sent_count += sent_count;
	g_msg_vec[i].pending_count += pending_count;
	g_msg_vec[i].on_hold_count += on_hold_count;
	if (pending_count == -1 && sent_count == 0)
		g_msg_vec[i].deleted_count++;
	else if (pending_count == -1 && sent_count == 1)
		g_msg_vec[i].pending_sent_count++;
	else if (on_hold_count == -1 && sent_count == 1)
		g_msg_vec[i].on_hold_sent_count++;
	else if (on_hold_count == -1 && pending_count == 1)
		g_msg_vec[i].on_hold_pending_count++;
}


void
get_orig_module_from_message_log(string &orig_module, string &status, int *pending_count, int *on_hold_count, char *line_c, int *pos)
{
	char orig_c[2000], rest[2000];

	if ((sscanf(line_c, " %*s %*s {%*d}: X_IPC Server {%*d} --> %n%s", pos, rest) == 1) ||
		(sscanf(line_c, " %*s %*s {%*d}: X_IPC Server --> %n%s", pos, rest) == 1))
	{
		orig_module = "X_IPC_Server";
	}
	else if ((sscanf(line_c, " %*s %*s {%*d}: %s {%*d} --> %n%s", orig_c, pos, rest) == 2) ||
			 (sscanf(line_c, " %*s %*s {%*d}: %s --> %n%s", orig_c, pos, rest) == 2))
	{
		orig_module = orig_c;
	}
	else if (sscanf(line_c, " %*s %*s {%*d}: Resource %*s --> %n%s", pos, rest) == 1)
	{
		status = "pending-->";
		*pending_count = -1;
	}
	else if (sscanf(line_c, " %*s %*s {%*d}: ON HOLD --> %n%s", pos, rest) == 1)
	{
		status = "on_hold-->";
		*on_hold_count = -1;
	}
	else
		orig_module = "???";
}


void
get_dest_module_from_message_log(string &dest_module, char *logtime, string &status, int *sent_count, int *pending_count, int *on_hold_count, char *line_c, int pos)
{
	char dest_c[2000];
	float logtime_secs;

	if (sscanf(&line_c[pos], " X_IPC Server (Sent) %*d:%*d:%f", &logtime_secs) == 1)
	{
		status += "sent";
		*sent_count = 1;
		dest_module = "X_IPC_Server";
		sscanf(&line_c[pos], " X_IPC Server (Sent) %s", logtime);
	}
	else if (sscanf(&line_c[pos], " %s (Sent) %*d:%*d:%f", dest_c, &logtime_secs) == 2)
	{
		status += "sent";
		*sent_count = 1;
		dest_module = dest_c;
		sscanf(&line_c[pos], " %*s (Sent) %s", logtime);
	}
	else if (sscanf(&line_c[pos], " %s {%*d} %*d:%*d:%f", dest_c, &logtime_secs) == 2)
	{
		status += "sent";
		*sent_count = 1;
		dest_module = dest_c;
		sscanf(&line_c[pos], " %*s {%*d} %s", logtime);
	}
	else if (sscanf(&line_c[pos], " %s %*d:%*d:%f", dest_c, &logtime_secs) == 2)
	{
		status += "sent";
		*sent_count = 1;
		dest_module = dest_c;
		sscanf(&line_c[pos], " %*s %s", logtime);
	}
	else if (sscanf(&line_c[pos], " Resource %s (Pending) %*d:%*d:%f", dest_c, &logtime_secs) == 2)
	{
		status += "pending";
		*pending_count = 1;
		dest_module = dest_c;
		sscanf(&line_c[pos], " Resource %*s (Pending) %s", logtime);
	}
	else if (sscanf(&line_c[pos], " ON HOLD (Inactive) %*d:%*d:%f", &logtime_secs) == 1)
	{
		status += "on_hold";
		*on_hold_count = 1;
		sscanf(&line_c[pos], " ON HOLD (Inactive) %s", logtime);
	}
	else
		dest_module = "???";
}


void
unknown_message_log(string line, long linecount, string line_data, bool filtered, int lines)
{
	if (filtered && filter_class(UNKNOWN_MESSAGE_CLASS))
	{
		if (g_verbose)
			printf("Logline: %9ld  Class: %-10s  Logtime: %s  Text: %s\n", linecount,
					g_log_record_class_name[UNKNOWN_MESSAGE_CLASS], g_logtime, line.c_str());
		if (g_verbose > 1 && lines > 0)
			printf("%s", line_data.c_str());
		update_logstats(UNKNOWN_MESSAGE_CLASS, 1);
		if (lines > 0)
			update_logstats(DATA_CLASS, lines);
	}
}


bool
get_token(string &token, string &text)
{
	size_t pos;

	if (text.length() == 0)
		return (false);

	if (text[0] == '*')
		pos = text.length();
	else if (strchr("{[<>]}", text[0]) != NULL)
		pos = 1;
	else if (text[0] == '"')
	{
		pos = text.find_first_of('"', 1);
		if (pos < text.length())
			pos++;
		if (pos < text.length() && strchr(",>]}", text[pos]) == NULL)
			return (false);
	}
	else
		pos = text.find_first_of(",>]}", 1);

	token = text.substr(0, pos);
	if (pos < text.length() && text[pos] == ',')
		pos = text.find_first_not_of(' ', pos + 1);
	text = (pos < text.length()) ? text.substr(pos) : "";

	return (true);
}


void
print_message_log_data(string unsplit_data)
{
	string data_token, data_text = unsplit_data;

	while(get_token(data_token, data_text))
		printf("\t%s", data_token.c_str());

	if (data_text.length())
		printf("\tUNFORMATTED: %s", data_text.c_str());
}


void
print_message_log(long linecount, char *msg_class_c, char *msg_name_c, int loc_id, string orig_module, string dest_module, string status, int lines, string line_data, string unsplit_data)
{
	if (g_tabular)
	{
		static bool first_time = true;
		if (first_time)
			printf("%9s  %-15s  %-35s  %3s  %-25s  %-25s  %-14s  %-13s %11s\t%s\n",
					"Logline", "Class", "Message", "Id", "Origin", "Destination", "Status", "Logtime", "Elapsedtime", "Data");
		first_time = false;

		printf("%9ld  %-15s  %-35s  %3d  %-25s  %-25s  %-14s  %-13s %11.3lf",
				linecount, msg_class_c, msg_name_c, loc_id, orig_module.c_str(), dest_module.c_str(), status.c_str(), g_logtime, elapsed_time(g_logtime));
		if (g_verbose > 1 && lines > 0)
			print_message_log_data(unsplit_data);
		printf("\n");
	}
	else
	{
		printf("Logline: %9ld  Class: %-15s  Message: %s  Id: %d  Origin: %s  Destination: %s  Status: %s  Logtime: %s (%.3lf)\n",
				linecount, msg_class_c, msg_name_c, loc_id, orig_module.c_str(), dest_module.c_str(), status.c_str(), g_logtime, elapsed_time(g_logtime));
		if (g_verbose > 1 && lines > 0)
			printf("%s", line_data.c_str());
	}
}


void
message_log(char *line_c, long *linecount, int *buffersize)
{
	long linecount_initial = *linecount;
	char msg_class_c[15] = {0}, msg_name_c[2000] = {0};
	int loc_id, pos = 0;
	int sent_count = 0, pending_count = 0, on_hold_count = 0;

	sscanf(line_c, " %s %s {%d}:", msg_class_c, msg_name_c, &loc_id);
	string orig_module, dest_module, status, line_data, unsplit_data, msg_name = msg_name_c, line = line_c;
	int msg_class_num = get_log_record_class(msg_class_c);
	bool filtered = filter_message(msg_name);

	get_orig_module_from_message_log(orig_module, status, &pending_count, &on_hold_count, line_c, &pos);
	get_dest_module_from_message_log(dest_module, g_logtime, status, &sent_count, &pending_count, &on_hold_count, line_c, pos);
	filtered &= filter_time(g_logtime);

	int lines = get_data(line_c, linecount, buffersize, line_data, unsplit_data);
	filtered &= filter_keyword(line + '\n' + unsplit_data);

	if (orig_module == "???" || dest_module == "???")
	{
		unknown_message_log(line, linecount_initial, line_data, filtered, lines);
		return;
	}

	if (msg_class_num == REPLY_CLASS && sent_count == 1)
	{
		string orig_query, dest_query, status_query;
		delete_pending_vec(msg_name, loc_id, orig_query, dest_query, status_query);
	}
	else
		update_pending_vec(msg_name, loc_id, orig_module, dest_module, status);

	filtered &= filter_module(orig_module, dest_module);
	filtered &= filter_class(msg_class_num);
	if (filtered)
	{
		if (g_verbose)
			print_message_log(linecount_initial, msg_class_c, msg_name_c, loc_id, orig_module, dest_module, status, lines, line_data, unsplit_data);
		update_message_vec(msg_name, orig_module, dest_module, sent_count, pending_count, on_hold_count);
		update_logstats(msg_class_num, 1);
		if (lines > 0)
			update_logstats(DATA_CLASS, lines);
	}
}


int
message_total_count (ipc_t msg_rec)
{
	int total_count = msg_rec.sent_count + msg_rec.deleted_count + msg_rec.pending_count + msg_rec.on_hold_count;

	return (total_count);
}


void
accum_stats(int *count, int *total, int *sent_sum, int *pend_sent_sum, int *del_sum, int *pend_sum,
		int *on_hold_sent_sum, int *on_hold_pend_sum, int *on_hold_sum, ipc_t rec)
{
	*count            += 1;
	*total            += message_total_count(rec);
	*sent_sum         += rec.sent_count;
	*del_sum          += rec.deleted_count;
	*pend_sum         += rec.pending_count;
	*on_hold_sum      += rec.on_hold_count;
	*pend_sent_sum    += rec.pending_sent_count;
	*on_hold_sent_sum += rec.on_hold_sent_count;
	*on_hold_pend_sum += rec.on_hold_pending_count;
}


enum message_sort_option {SORT_MSG_BY_ORIG, SORT_MSG_BY_DEST};

void
print_message_summary(int sort_option = 0)
{
	int count  = 0, total  = 0, sent_sum  = 0, pend_sent_sum  = 0, del_sum  = 0, pend_sum  = 0, on_hold_sent_sum  = 0, on_hold_pend_sum  = 0, on_hold_sum  = 0;
	int count1 = 0, total1 = 0, sent_sum1 = 0, pend_sent_sum1 = 0, del_sum1 = 0, pend_sum1 = 0, on_hold_sent_sum1 = 0, on_hold_pend_sum1 = 0, on_hold_sum1 = 0;
	int count2 = 0, total2 = 0, sent_sum2 = 0, pend_sent_sum2 = 0, del_sum2 = 0, pend_sum2 = 0, on_hold_sent_sum2 = 0, on_hold_pend_sum2 = 0, on_hold_sum2 = 0;
	string sort_key1 = "\n", sort_key2 = "\n", current_key1, current_key2, current_detail;
	const char *sort_title[] = {"Origin", "Destination"};
	bool (*sort_order_function[]) (ipc_t, ipc_t) = {msg_order_by_orig, msg_order_by_dest};
	int so = (sort_option == 0) ? 0 : 1;

	printf("\n========================================================================"
		   "========================================================================\n%s %s"
		   "\n========================================================================"
		   "========================================================================\n", "Message Summary by", sort_title[so]);
	sort(g_msg_vec.begin(), g_msg_vec.end(), sort_order_function[so]);
	for (int i = 0; i < (int) g_msg_vec.size(); i++)
	{
		current_key1   = g_msg_vec[i].msg_name;
		current_key2   = (so == 0) ? g_msg_vec[i].orig_module : g_msg_vec[i].dest_module;
		current_detail = (so == 0) ? g_msg_vec[i].dest_module : g_msg_vec[i].orig_module;
		if (sort_key1 != current_key1 || sort_key2 != current_key2)
		{
			if (i > 0)
				printf("\t%-37s  %9d  %9d  %9d  %9d  %9d  %9d  %9d  %9d  %9d\n", (sort_title[so] + string(" Subtotals:")).c_str(), count2, total2,
						sent_sum2, del_sum2, pend_sum2, on_hold_sum2, pend_sent_sum2, on_hold_sent_sum2, on_hold_pend_sum2);
			sort_key2 = current_key2;
			count2 = 0, total2 = 0, sent_sum2 = 0, pend_sent_sum2 = 0, del_sum2 = 0, pend_sum2 = 0, on_hold_sent_sum2 = 0, on_hold_pend_sum2 = 0, on_hold_sum2 = 0;
			if (sort_key1 != current_key1)
			{
				if (i > 0)
					printf("%-45s  %9d  %9d  %9d  %9d  %9d  %9d  %9d  %9d  %9d\n", "Message Subtotals:", count1, total1,
							sent_sum1, del_sum1, pend_sum1, on_hold_sum1, pend_sent_sum1, on_hold_sent_sum1, on_hold_pend_sum1);
				sort_key1 = current_key1;
				count1 = 0, total1 = 0, sent_sum1 = 0, pend_sent_sum1 = 0, del_sum1 = 0, pend_sum1 = 0, on_hold_sent_sum1 = 0, on_hold_pend_sum1 = 0, on_hold_sum1 = 0;
				printf("\n*** Message: %s\n", sort_key1.c_str());
			}
			printf("\t%s: %s\n", sort_title[so], sort_key2.c_str());
			printf("\t\t%-40s      total       sent    deleted    pending    on hold  pend>sent  hold>sent  hold>pend\n", sort_title[1 - so]);
			printf("\t\t----------------------------------------" "  ---------  ---------  ---------  ---------  ---------  ---------  ---------  ---------\n");
		}
		printf("\t\t%-40s  %9d  %9d  %9d  %9d  %9d  %9d  %9d  %9d\n", current_detail.c_str(), message_total_count(g_msg_vec[i]),
				g_msg_vec[i].sent_count, g_msg_vec[i].deleted_count, g_msg_vec[i].pending_count, g_msg_vec[i].on_hold_count,
				g_msg_vec[i].pending_sent_count, g_msg_vec[i].on_hold_sent_count, g_msg_vec[i].on_hold_pending_count);
		accum_stats(&count,  &total,  &sent_sum,  &pend_sent_sum,  &del_sum,  &pend_sum,  &on_hold_sent_sum,  &on_hold_pend_sum,  &on_hold_sum,  g_msg_vec[i]);
		accum_stats(&count1, &total1, &sent_sum1, &pend_sent_sum1, &del_sum1, &pend_sum1, &on_hold_sent_sum1, &on_hold_pend_sum1, &on_hold_sum1, g_msg_vec[i]);
		accum_stats(&count2, &total2, &sent_sum2, &pend_sent_sum2, &del_sum2, &pend_sum2, &on_hold_sent_sum2, &on_hold_pend_sum2, &on_hold_sum2, g_msg_vec[i]);
		if (i == (int) g_msg_vec.size() - 1)
		{
			printf("\t%-37s  %9d  %9d  %9d  %9d  %9d  %9d  %9d  %9d  %9d\n", (sort_title[so] + string(" Subtotals:")).c_str(), count2, total2,
					sent_sum2, del_sum2, pend_sum2, on_hold_sum2, pend_sent_sum2, on_hold_sent_sum2, on_hold_pend_sum2);
			printf("%-45s  %9d  %9d  %9d  %9d  %9d  %9d  %9d  %9d  %9d\n", "Message Subtotals:", count1, total1,
					sent_sum1, del_sum1, pend_sum1, on_hold_sum1, pend_sent_sum1, on_hold_sent_sum1, on_hold_pend_sum1);
			printf("\n%-45s  %9d  %9d  %9d  %9d  %9d  %9d  %9d  %9d  %9d\n", "Grand Totals:", count, total,
					sent_sum, del_sum, pend_sum, on_hold_sum, pend_sent_sum, on_hold_sent_sum, on_hold_pend_sum);
			printf("========================================================================"
				   "========================================================================\n\n");
		}
	}
}


void
broadcast_log(char *line_c, long *linecount, int *buffersize)
{
	message_log(line_c, linecount, buffersize);
}


void
deleted_log(char *line_c, long linecount, char *msg_name_c, int loc_id)
{
	string msg_name = msg_name_c, orig_module, dest_module, status, line = line_c;
	bool filtered = filter_class(DELETED_CLASS);
	filtered &= filter_time(g_logtime);
	filtered &= filter_keyword(line);
	filtered &= filter_message(msg_name);

	delete_pending_vec(msg_name, loc_id, orig_module, dest_module, status);

	filtered &= filter_module(orig_module, dest_module);
	if (filtered)
	{
		if (g_verbose)
			printf("Logline: %9ld  Class: %-10s  Message: %s  Id: %d  Origin: %s  Destination: %s  Status: %s  Logtime: %s\n", linecount,
					g_log_record_class_name[DELETED_CLASS], msg_name_c, loc_id, orig_module.c_str(), dest_module.c_str(), status.c_str(), g_logtime);
		update_message_vec(msg_name, orig_module, dest_module, 0, -1, 0);
		update_logstats(DELETED_CLASS, 1);
	}
}


void
inform_log(char *line_c, long *linecount, int *buffersize)
{
	message_log(line_c, linecount, buffersize);
}


void
other_log(int msg_class_num, char *msg_subclass_c, char *line_c, long linecount)
{
	bool filtered = filter_class(msg_class_num);
	filtered &= filter_time(g_logtime);
	filtered &= filter_keyword(string(line_c));
	filtered &= filter_message(string(""));
	filtered &= filter_module(string(""), string(""));

	if (filtered)
	{
		if (g_verbose)
			printf("Logline: %9ld  Class: %s%-9s  Logtime: %-12s  Text: %s\n", linecount,
					g_log_record_class_name[msg_class_num], msg_subclass_c, g_logtime, line_c);
		update_logstats(msg_class_num, 1);
	}
}


void
unknown_log(char *line_c, long linecount)
{
	other_log(UNKNOWN_CLASS, (char *) "", line_c, linecount);
}


bool
done_log(char *line_c, long linecount, string full_line = "")
{
	char msg_name_c[2000] = {0};
	int loc_id;
	float logtime_secs;

	if (sscanf(line_c, " Done %s {%d}: %*d:%*d:%f", msg_name_c, &loc_id, &logtime_secs) != 3)
	{
		unknown_log(line_c, linecount);
		return (false);
	}
	sscanf(line_c, " Done %*s {%*d}: %s", g_logtime);

	string orig_module, dest_module, status, line = line_c, msg_name = msg_name_c;
	bool filtered = filter_class(DONE_CLASS);
	filtered &= filter_time(g_logtime);
	filtered &= (filter_keyword(line) || filter_keyword(full_line));
	filtered &= filter_message(msg_name);

	delete_pending_vec(msg_name, loc_id, orig_module, dest_module, status);

	filtered &= filter_module(orig_module, dest_module);
	if (filtered)
	{
		if (g_verbose)
			printf("Logline: %9ld  Class: %-10s  Message: %s  Id: %d  Origin: %s  Destination: %s  Status: %s  Logtime: %s\n", linecount,
					g_log_record_class_name[DONE_CLASS], msg_name_c, loc_id, orig_module.c_str(), dest_module.c_str(), status.c_str(), g_logtime);
		update_logstats(DONE_CLASS, 1);
	}
	return (filtered);
}


void
query_log(char *line_c, long *linecount, int *buffersize)
{
	message_log(line_c, linecount, buffersize);
}


void
reply_log(char *line_c, long *linecount, int *buffersize)
{
	message_log(line_c, linecount, buffersize);
}


void
transfer_log(char *line_c, long linecount)
{
	char msg_name_c[2000] = {0}, new_dest_module_c[2000] = {0}, logtime[200] = {0};
	int loc_id, pos = 0, pos_scan;

	while (sscanf(&line_c[pos], " Transferring %s {%d} from Resource %*s to %s%n", msg_name_c, &loc_id, new_dest_module_c, &pos_scan) == 3)
		pos += pos_scan;

	if (sscanf(&line_c[pos], " Done %s {%d}: %s", msg_name_c, &loc_id, logtime) != 3)
	{
		unknown_log(line_c, linecount);
		return;
	}
	if (!done_log(&line_c[pos], linecount, string(line_c)))
		return;

	pos = 0;
	while (sscanf(&line_c[pos], " Transferring %s {%d} from Resource %*s to %s%n", msg_name_c, &loc_id, new_dest_module_c, &pos_scan) == 3)
	{
		string orig_module = "???", old_dest_module = "???", new_dest_module = new_dest_module_c, msg_name = msg_name_c;
		bool filtered = filter_keyword(string(&line_c[pos], pos_scan));
		filtered &= filter_message(msg_name);
		pos += pos_scan;

		int p = locate_pending(msg_name, loc_id);
		if (p >= 0)
		{
			orig_module = g_pending_vec[p].orig_module;
			old_dest_module = g_pending_vec[p].dest_module;
			g_pending_vec[p].dest_module = new_dest_module;

			int i_old = locate_message(msg_name, orig_module, old_dest_module);
			if (i_old >= 0)
			{
				g_msg_vec[i_old].pending_count--;
				if (message_total_count(g_msg_vec[i_old]) == 0)
					g_msg_vec.erase(g_msg_vec.begin() + i_old);
			}

			int i_new = locate_message(msg_name, orig_module, new_dest_module);
			if (i_new < 0)
			{
				ipc_t message = {msg_name, orig_module, new_dest_module, 0, 0, 0, 0, 0, 0, 0};
				g_msg_vec.push_back(message);
				i_new = g_msg_vec.size() - 1;
			}
			g_msg_vec[i_new].pending_count++;
		}

		filtered &= (filter_module(orig_module, old_dest_module) || filter_module(orig_module, new_dest_module));
		if (filtered && g_verbose)
			printf("Logline: %9ld  Class: %s%-9s  Message: %s  Id: %d  Origin: %s  Destination: %s > %s  Logtime: %s\n", linecount,
				g_log_record_class_name[DONE_CLASS], "/Transfer", msg_name_c, loc_id, orig_module.c_str(), old_dest_module.c_str(), new_dest_module_c, g_logtime);
	}
}


void
clear_log(char *line_c, long linecount, char *msg_name_c, int loc_id)
{
	string orig_module, dest_module, status, msg_name = msg_name_c, line = line_c;
	bool filtered = filter_class(OTHER_CLASS);
	filtered &= filter_time(g_logtime);
	filtered &= filter_keyword(line);
	filtered &= filter_message(msg_name);

	delete_pending_vec(msg_name, loc_id, orig_module, dest_module, status);

	filtered &= filter_module(orig_module, dest_module);
	if (filtered)
	{
		if (g_verbose)
			printf("Logline: %9ld  Class: %s%-9s  Message: %s  Id: %d  Origin: %s  Destination: %s  Status: %s  Logtime: %s\n", linecount,
					g_log_record_class_name[OTHER_CLASS], "/Clear", msg_name_c, loc_id, orig_module.c_str(), dest_module.c_str(), status.c_str(), g_logtime);
		update_logstats(OTHER_CLASS, 1);
	}
}


void
registration_log(char *line_c, long linecount, char *msg_name_c)
{
	string msg_name = msg_name_c, line = line_c;
	bool filtered = filter_class(REGISTRATION_CLASS);
	filtered &= filter_time(g_logtime);
	filtered &= filter_keyword(line);
	filtered &= filter_message(msg_name);
	filtered &= filter_module(string(""), string(""));

	if (filtered)
	{
		if (g_verbose)
			printf("Logline: %9ld  Class: %-10s  Message: %s  Logtime: %s\n", linecount,
					g_log_record_class_name[REGISTRATION_CLASS], msg_name_c, g_logtime);
		update_msg_reg_vec(msg_name, 1);
		update_logstats(REGISTRATION_CLASS, 1);
	}
}


void
connection_log(char *line_c, long *linecount, int *buffersize, int connection_id)
{
	int lines = 0;
	string line = line_c;
	char module_c[2000] = {0}, host_c[2000] = {0};
	bool filtered = filter_class(CONNECTION_CLASS);
	filtered &= filter_time(g_logtime);
	filtered &= filter_message(string(""));

	long textstart = ftell(g_logfile);
	if (read_line(g_logfile, line_c, buffersize))
	{
		if (sscanf(line_c, " modName : %s", module_c) == 1)
		{
			lines++;
			line += '\n' + line_c;
		}
		else
			fseek(g_logfile, textstart, SEEK_SET);
	}

	textstart = ftell(g_logfile);
	if (read_line(g_logfile, line_c, buffersize))
	{
		if (sscanf(line_c, " hostName: %s", host_c) == 1)
		{
			lines++;
			line += '\n' + line_c;
		}
		else
			fseek(g_logfile, textstart, SEEK_SET);
	}

	string module = module_c;
	filtered &= filter_module(module);
	filtered &= filter_keyword(line);
	if (filtered)
	{
		if (g_verbose)
			printf("Logline: %9ld  Class: %s%-6s  Connection id: %2d  Module: %s  Hostname: %s  Logtime: %s\n", *linecount,
					g_log_record_class_name[CONNECTION_CLASS], "/Open", connection_id, module_c, host_c, g_logtime);
		update_module_vec(module, 1, 0);
		update_logstats(CONNECTION_CLASS, 1 + lines);
	}
	*linecount += lines;
}


void
close_log(char *line_c, long *linecount, int *buffersize,  int connection_id, char *module_c, char *host_c)
{
	int lines = 0;
	string line = line_c;
	bool filtered = filter_class(CONNECTION_CLASS);
	filtered &= filter_time(g_logtime);
	filtered &= filter_message(string(""));

	long textstart = ftell(g_logfile);
	if (read_line(g_logfile, line_c, buffersize))
	{
		if (sscanf(line_c, " Closing %s on %s", module_c, host_c) == 2 ||
			sscanf(line_c, " x_ipcClose: Closing %s on %s", module_c, host_c) == 2)
		{
			lines++;
			line += '\n' + line_c;
		}
		else
			fseek(g_logfile, textstart, SEEK_SET);
	}

	textstart = ftell(g_logfile);
	if (read_line(g_logfile, line_c, buffersize))
	{
		if (sscanf(line_c, " close Module: Closing %s", module_c) == 1)
		{
			lines++;
			line += '\n' + line_c;
		}
		else
			fseek(g_logfile, textstart, SEEK_SET);
	}

	string module = module_c;
	filtered &= filter_module(module);
	filtered &= filter_keyword(line);
	if (filtered)
	{
		if (g_verbose)
			printf("Logline: %9ld  Class: %s%-6s  Connection id: %2d  Module: %s  Hostname: %s  Logtime: %s\n", *linecount,
					g_log_record_class_name[CONNECTION_CLASS], "/Close", connection_id, module_c, host_c, g_logtime);
		update_module_vec(module, 0, 1);
		update_logstats(CONNECTION_CLASS, 1 + lines);
	}
	*linecount += lines;
}


void
handle_summary_log(char *line_c, long linecount)
{
	long mTotal, byteSize, diffTime, avgTime;
	float monPer;
	bool filtered = filter_class(HANDLE_SUMMARY_CLASS);
	filtered &= filter_time(g_logtime);
	filtered &= filter_keyword(string(line_c));
	filtered &= filter_message(string(""));
	filtered &= filter_module(string(""), string(""));

	sscanf(line_c, " %ld: Msg: %f%%, Mon: %f%%, Wait %f%%, Msg: (%ld, %ld) Ave: (%ld, %ld)",
			&mTotal, &g_msgPer, &monPer, &g_waitPer, &byteSize, &diffTime, &g_avgSize, &avgTime);

	if (filtered)
	{
		if (g_verbose)
			printf("Logline: %9ld  Class: %-10s  Logtime: %-12s  Text: %s\n", linecount,
					g_log_record_class_name[HANDLE_SUMMARY_CLASS], g_logtime, line_c);
		update_logstats(HANDLE_SUMMARY_CLASS, 1);
	}
}


void
start_log(char *line_c, long linecount, char *logstarttime)
{
	printf("Log start time: %s\n\n", logstarttime);

	float logtime_secs;
	if (sscanf(logstarttime, " %*s %*s %*d %*d:%*d:%f", &logtime_secs) == 1)
		sscanf(logstarttime, " %*s %*s %*d %s", g_logtime);

	other_log(OTHER_CLASS, (char *) "/Start", line_c, linecount);
}


void
expect_log(char *line_c, long linecount)
{
	other_log(OTHER_CLASS, (char *) "/Expect", line_c, linecount);
}


void
abort_log(char *line_c, long linecount)
{
	other_log(OTHER_CLASS, (char *) "/Abort", line_c, linecount);
}


void
process_analysis()
{
	long linecount = 0;
	int buffersize = 2000;
	char *line_c = (char *) malloc(buffersize + 1);
	char msg_name_c[2000], module_c[2000], host_c[2000], rest[2000];

	while (read_line(g_logfile, line_c, &buffersize))
	{
		int id = -1, conn_id = -1;
		module_c[0] = host_c[0] = 0;
		linecount++;

		if (sscanf(line_c, " Broadcast %s {%d}:", msg_name_c, &id) == 2)
			broadcast_log(line_c, &linecount, &buffersize);
		else if (sscanf(line_c, " PENDING LIMIT: Deleted message %s {%d}", msg_name_c, &id) == 2)
			deleted_log(line_c, linecount, msg_name_c, id);
		else if (sscanf(line_c, " Inform %s {%d}:", msg_name_c, &id) == 2)
			inform_log(line_c, &linecount, &buffersize);
		else if (sscanf(line_c, " Done %s {%d}:", msg_name_c, &id) == 2)
			done_log(line_c, linecount);
		else if (sscanf(line_c, " Query %s {%d}:", msg_name_c, &id) == 2)
			query_log(line_c, &linecount, &buffersize);
		else if (sscanf(line_c, " Reply %s {%d}:", msg_name_c, &id) == 2)
			reply_log(line_c, &linecount, &buffersize);
		else if (sscanf(line_c, " Transferring %s {%d}", msg_name_c, &id) == 2)
			transfer_log(line_c, linecount);
		else if (sscanf(line_c, " Clearing handler for message %s {%d}", msg_name_c, &id) == 2)
			clear_log(line_c, linecount, msg_name_c, id);
		else if (sscanf(line_c, " Registration: Message %s Found. Updating.", msg_name_c) == 1)
			registration_log(line_c, linecount, msg_name_c);
		else if (sscanf(line_c, " Received a new connection: %d", &conn_id) == 1)
			connection_log(line_c, &linecount, &buffersize, conn_id);
		else if (sscanf(line_c, " Closed Connection %*s %*s %*s %d", &conn_id) == 1 || sscanf(line_c, " %*s Closed Connection %*s %*s %*s %d", &conn_id) == 1 ||
				 sscanf(line_c, " Closing %s on %s", module_c, host_c) == 2 || sscanf(line_c, " %*s Closing %s on %s", module_c, host_c) == 2)
			close_log(line_c, &linecount, &buffersize, conn_id, module_c, host_c);
		else if (sscanf(line_c, " %*d: Msg: %s", rest) == 1)
			handle_summary_log(line_c, linecount);
		else if (sscanf(line_c, " Logging Task Control Server %*s %*s on %[^\n]", g_logstarttime) == 1)
			start_log(line_c, linecount, g_logstarttime);
		else if (sscanf(line_c, " Expecting %s", rest) == 1)
			expect_log(line_c, linecount);
		else if (sscanf(line_c, " Central Abort : %s", rest) == 1)
			abort_log(line_c, linecount);
		else
			unknown_log(line_c, linecount);
	}
	free(line_c);
}


void
usage()
{
	fprintf(stderr, "\nUsage: central_log_view <logfile> [args]\n" " args:\n"
		"    -v <verbose>   : verbose option: 0 = summary only,\n"
		"                     1 = all except data lines, 2 = all (default: 0)\n"
		"    -vt <verbose>  : verbose option in tabular format\n"
		"    -s <summaries> : summary option: c = Connection, r = Registration,\n"
		"                     o = Message by Origin, d = Message by Destination,\n"
		"                     * = crod (default: general summary)\n"
		"    -t <t1> <t2>   : time filter: from <t1> to <t2>, format hh:mm:ss.cc\n"
		"                     (default: 00:00 23:59:59.99)\n"
		"    -c <classes>   : class filter: c = Connection, r = Registration,\n"
		"                     b = Broadcast, i = Inform, d = Done, q = Query, p = Reply,\n"
		"                     x = Deleted, h = Handle, o = Other, u = Unknown,\n"
		"                     * = crbidqpxhou (default = *)\n"
		"    -msg <msgs>    : message filter: list of tokens separated by spaces\n"
		"                     wildcards allowed (default: *)\n"
		"    -mod <modules> : module filter: list of tokens separated by spaces\n"
		"                     wildcards allowed (default: *)\n"
		"    -mod <m1>:<m2> : module filter: origin module <m1>, destination module <m2>\n"
		"                     list separated by spaces, wildcards allowed (default: *:*)\n"
		"    -k <keywords>  : keyword filter: list of tokens separated by spaces\n"
		"                     wildcards allowed (default: *)\n"
		"    -i<filter>     : inverted filters: -imsg, -imod, -ik\n\n"
		"    Warning: tokens that include special characters *?-#&;~()|<>`\\\"'<space>\n"
		"             might require to be escaped with either \\ or \"\" or ''\n\n");
}


void
print_summary()
{
	print_general_summary();
	if (g_summary_connection)
		print_module_summary();
	if (g_summary_registration)
		print_registration_summary();
	if (g_summary_orig_message)
		print_message_summary(SORT_MSG_BY_ORIG);
	if (g_summary_dest_message)
		print_message_summary(SORT_MSG_BY_DEST);
	if (g_print_usage)
		usage();
}


void
terminate(char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);

	usage();
	std::exit(-1);
}


void
read_verbose_option(int *verbose, bool *tabular, int *arg_num, int argc, char **argv)
{
	char *option = argv[*arg_num];
	*tabular = (option[2] == 't');

	static bool first_time = true;
	if (!first_time)
		terminate((char *) "arg[%d]: Only one verbose option -v is allowed\n", *arg_num);
	first_time = false;

	if ((*arg_num == argc - 1) || (argv[*arg_num + 1][0] == '-'))
		terminate((char *) "arg[%d]: Verbose option expected after %s\n", *arg_num, option);
	*arg_num += 1;
	char *param = argv[*arg_num];
	int pos = 0;

	if ((sscanf(param, "%d %n", verbose, &pos) != 1) || (param[pos] != 0))
		terminate((char *) "arg[%d]: Invalid verbose option: %s\n", *arg_num, param);

	if (*verbose < 0 || *verbose > 2)
		terminate((char *) "arg[%d]: Invalid verbose option: %s\n", *arg_num, param);
}


void
read_summary_option(bool *connection, bool *registration, bool *orig_message, bool *dest_message, int *arg_num, int argc, char **argv)
{
	if ((*arg_num == argc - 1) || (argv[*arg_num + 1][0] == '-'))
		terminate((char *) "arg[%d]: Summary option expected after -s\n", *arg_num);
	*arg_num += 1;
	char *param = argv[*arg_num];

	static bool first_time = true;
	if (first_time)
		*connection = *registration = *orig_message = *dest_message = false;
	first_time = false;

	for (int i = 0; param[i] != 0; i++)
	{
		if (param[i] == 'c')
			*connection = true;
		else if (param[i] == 'r')
			*registration = true;
		else if (param[i] == 'o')
			*orig_message = true;
		else if (param[i] == 'd')
			*dest_message = true;
		else if (param[i] == '*')
			*connection = *registration = *orig_message = *dest_message = true;
		else
			terminate((char *) "arg[%d]: Invalid summary option: %s\n", *arg_num, param);
	}
}


void
read_time_filter(double *time_initial, double *time_final, int *arg_num, int argc, char **argv)
{
	static bool first_time = true;
	if (!first_time)
		terminate((char *) "arg[%d]: Only one time filter -t is allowed\n", *arg_num);
	first_time = false;

	if ((*arg_num == argc - 1) || (argv[*arg_num + 1][0] == '-'))
		terminate((char *) "arg[%d]: Time parameter expected after -t\n", *arg_num);
	*arg_num += 1;
	char *param = argv[*arg_num];
	*time_initial = time_str2dbl(param);
	if (*time_initial < 0.0)
		terminate((char *) "arg[%d]: Invalid time parameter value: %s\n", *arg_num, param);

	if ((*arg_num == argc - 1) || (argv[*arg_num + 1][0] == '-'))
		*time_final = (24.0 * 60.0 * 60.0 - 0.01);
	else
	{
		*arg_num += 1;
		param = argv[*arg_num];
		*time_final = time_str2dbl(param);
		if (*time_final < 0.0)
			terminate((char *) "arg[%d]: Invalid time parameter value: %s\n", *arg_num, param);
		if (*time_initial > *time_final)
			terminate((char *) "arg[%d]: The final time parameter must be after the initial time: %s\n", *arg_num, param);
	}

	g_filter_time_all =  (*time_initial == 0.0 && *time_final >= (24.0 * 60.0 * 60.0 - 0.01));
}


void
read_class_filter(bool *class_array, int class_size, int *arg_num, int argc, char **argv)
{
	if ((*arg_num == argc - 1) || (argv[*arg_num + 1][0] == '-'))
		terminate((char *) "arg[%d]: Class parameter expected after -c\n", *arg_num);
	*arg_num += 1;
	char *param = argv[*arg_num];

	static bool first_time = true;
	if (first_time)
	{
		g_filter_class_all = false;
		for (int k = 0; k < class_size; k++)
			class_array[k] = false;
	}
	first_time = false;

	for (int i = 0; param[i] != 0; i++)
	{
		if (param[i] == 'c')
			class_array[CONNECTION_CLASS] = true;
		else if (param[i] == 'r')
			class_array[REGISTRATION_CLASS] = true;
		else if (param[i] == 'b')
			class_array[BROADCAST_CLASS] = class_array[DATA_CLASS] = true;
		else if (param[i] == 'i')
			class_array[INFORM_CLASS] = class_array[DATA_CLASS] = true;
		else if (param[i] == 'd')
			class_array[DONE_CLASS] = true;
		else if (param[i] == 'q')
			class_array[QUERY_CLASS] = class_array[DATA_CLASS] = true;
		else if (param[i] == 'p')
			class_array[REPLY_CLASS] = class_array[DATA_CLASS] = true;
		else if (param[i] == 'x')
			class_array[DELETED_CLASS] = true;
		else if (param[i] == 'h')
			class_array[HANDLE_SUMMARY_CLASS] = true;
		else if (param[i] == 'o')
			class_array[OTHER_CLASS] = true;
		else if (param[i] == 'u')
			class_array[UNKNOWN_MESSAGE_CLASS] = class_array[UNKNOWN_CLASS] = true;
		else if (param[i] == '*')
			g_filter_class_all = true;
		else
			terminate((char *) "arg[%d]: Invalid class parameter: %s\n", *arg_num, param);
	}

	if (!g_filter_class_all)
	{
		g_filter_class_all = true;
		for (int k = 0; k < class_size; k++)
			g_filter_class_all &= class_array[k];
	}
}


void
read_message_filter(vector<string> &msg_vec, vector<string> &inv_msg_vec, int *arg_num, int argc, char **argv)
{
	if ((*arg_num == argc - 1) || (argv[*arg_num + 1][0] == '-'))
		terminate((char *) "arg[%d]: Message parameter expected after -msg\n", *arg_num);

	char *filter = argv[*arg_num];
	bool inv_filter = (filter[1] == 'i');
	bool msg_all = false;
	static bool first_time = true;
	if (first_time)
		g_filter_msg_all = false;
	first_time = false;

	for (*arg_num += 1; *arg_num < argc; *arg_num += 1)
	{
		char *param = argv[*arg_num];
		vector<string> &token_vec = (inv_filter) ? inv_msg_vec : msg_vec;
		token_vec.push_back(param);

		if (string(param) == "*")
		{
			printf("arg[%d]: Warning: using parameter '%s' in filter %s\n", *arg_num, param, filter);
			msg_all |= !inv_filter;
		}

		if ((*arg_num == argc - 1) || (argv[*arg_num + 1][0] == '-'))
			break;
	}
	g_filter_msg_all = (g_filter_msg_all || msg_all) && inv_msg_vec.empty();
}


void
read_module_filter(vector<string> &orig_vec, vector<string> &dest_vec, vector<string> &inv_orig_vec, vector<string> &inv_dest_vec,
		int *arg_num, int argc, char **argv)
{
	if ((*arg_num == argc - 1) || (argv[*arg_num + 1][0] == '-'))
		terminate((char *) "arg[%d]: Module parameter expected after -mod\n", *arg_num);

	char *filter = argv[*arg_num];
	bool inv_filter = (filter[1] == 'i');
	bool mod_all = false;
	static bool first_time = true;
	if (first_time)
		g_filter_mod_all = false;
	first_time = false;

	for (*arg_num += 1; *arg_num < argc; *arg_num += 1)
	{
		char *param = argv[*arg_num], orig_module[2000] = {0}, dest_module[2000] = {0};
		vector<string> &token1_vec = (inv_filter) ? inv_orig_vec : orig_vec;
		vector<string> &token2_vec = (inv_filter) ? inv_dest_vec : dest_vec;

		if (sscanf(param, "%[^:]:%[^:]", orig_module, dest_module) == 2)
		{
			token1_vec.push_back(orig_module);
			token2_vec.push_back(dest_module);
		}
		else
		{
			token1_vec.push_back(param);
			token2_vec.push_back("*");
			token1_vec.push_back("*");
			token2_vec.push_back(param);
		}

		if (string(param) == "*:*" || string(param) == "*")
		{
			printf("arg[%d]: Warning: using parameter '%s' in filter %s\n", *arg_num, param, filter);
			mod_all |= !inv_filter;
		}

		if ((*arg_num == argc - 1) || (argv[*arg_num + 1][0] == '-'))
			break;
	}
	g_filter_mod_all = (g_filter_mod_all || mod_all) && inv_orig_vec.empty();
}


void
read_keyword_filter(vector<string> &key_vec, vector<string> &inv_key_vec, int *arg_num, int argc, char **argv)
{
	if ((*arg_num == argc - 1) || (argv[*arg_num + 1][0] == '-'))
		terminate((char *) "arg[%d]: Keyword parameter expected after -k\n", *arg_num);

	char *filter = argv[*arg_num];
	bool inv_filter = (filter[1] == 'i');
	bool keyword_all = false;
	static bool first_time = true;
	if (first_time)
		g_filter_keyword_all = false;
	first_time = false;

	for (*arg_num += 1; *arg_num < argc; *arg_num += 1)
	{
		char *param = argv[*arg_num];
		vector<string> &token_vec = (inv_filter) ? inv_key_vec : key_vec;
		token_vec.push_back(param);

		if (string(param) == "*")
		{
			printf("arg[%d]: Warning: using parameter '%s' in filter %s\n", *arg_num, param, filter);
			keyword_all |= !inv_filter;
		}

		if ((*arg_num == argc - 1) || (argv[*arg_num + 1][0] == '-'))
			break;
	}
	g_filter_keyword_all = (g_filter_keyword_all || keyword_all) && inv_key_vec.empty();
}


void
read_parameters(int argc, char **argv)
{
	printf("Commandline: ");
	for (int i = 0; i < argc; i++)
		printf("%s ", argv[i]);
	printf("\n\nCentral Log View Utility\n" "Log file: %s\n", g_logfile_name);

	for (int i = 2; i < argc; i++)
	{
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "-vt") == 0)
			read_verbose_option(&g_verbose, &g_tabular, &i, argc, argv);
		else if (strcmp(argv[i], "-s") == 0)
			read_summary_option(&g_summary_connection, &g_summary_registration, &g_summary_orig_message, &g_summary_dest_message, &i, argc, argv);
		else if (strcmp(argv[i], "-t") == 0)
			read_time_filter(&g_filter_time_initial, &g_filter_time_final, &i, argc, argv);
		else if (strcmp(argv[i], "-c") == 0)
			read_class_filter(g_filter_class, LOG_RECORD_CLASS_SIZE, &i, argc, argv);
		else if (strcmp(argv[i], "-msg") == 0 || strcmp(argv[i], "-imsg") == 0)
			read_message_filter(g_filter_msg_vec, g_filter_inv_msg_vec, &i, argc, argv);
		else if (strcmp(argv[i], "-mod") == 0 || strcmp(argv[i], "-imod") == 0)
			read_module_filter(g_filter_orig_vec, g_filter_dest_vec, g_filter_inv_orig_vec, g_filter_inv_dest_vec, &i, argc, argv);
		else if (strcmp(argv[i], "-k") == 0 || strcmp(argv[i], "-ik") == 0)
			read_keyword_filter(g_filter_key_vec, g_filter_inv_key_vec, &i, argc, argv);
		else
			terminate((char *) "arg[%d]: Invalid command line argument: %s\n", i, argv[i]);
	}
	if (argc == 2)
		g_print_usage = true;
}


int
main(int argc, char **argv)
{
	if (argc < 2)
		terminate((char *) "Error: wrong number of parameters: program requires 1 mandatory parameter and received %d parameter(s).\n"
				"Usage: %s <logfile>\n", argc - 1, argv[0]);
	if (strcmp(argv[1], "-h") == 0)
		terminate(NULL);

    g_logfile_name = argv[1];
	g_logfile = fopen(g_logfile_name, "r");
	if (g_logfile == NULL)
		terminate((char *) "Error: could not open file %s for reading.\n", g_logfile_name);

	read_parameters(argc, argv);
	process_analysis();
	print_summary();
	fclose(g_logfile);

	return (0);
}
