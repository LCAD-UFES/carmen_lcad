/**
 * SyntaxHighlighter
 * http://alexgorbatchev.com/
 *
 * SyntaxHighlighter is donationware. If you are using it, please donate.
 * http://alexgorbatchev.com/wiki/SyntaxHighlighter:Donate
 *
 * @version
 * 2.0.320 (May 03 2009)
 * 
 * @copyright
 * Copyright (C) 2004-2009 Alex Gorbatchev.
 *
 * @license
 * This file is part of SyntaxHighlighter.
 * 
 * SyntaxHighlighter is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * SyntaxHighlighter is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with SyntaxHighlighter.  If not, see <http://www.gnu.org/copyleft/lesser.html>.
 */
SyntaxHighlighter.brushes.Oj = function()
{
	// Copyright 2006 Shin, YoungJin
	
	var datatypes =	'ATOM BOOL BOOLEAN BYTE CHAR COLORREF DWORD DWORDLONG DWORD_PTR ' +
					'DWORD32 DWORD64 FLOAT HACCEL HALF_PTR HANDLE HBITMAP HBRUSH ' +
					'HCOLORSPACE HCONV HCONVLIST HCURSOR HDC HDDEDATA HDESK HDROP HDWP ' +
					'HENHMETAFILE HFILE HFONT HGDIOBJ HGLOBAL HHOOK HICON HINSTANCE HKEY ' +
					'HKL HLOCAL HMENU HMETAFILE HMODULE HMONITOR HPALETTE HPEN HRESULT ' +
					'HRGN HRSRC HSZ HWINSTA HWND INT INT_PTR INT32 INT64 LANGID LCID LCTYPE ' +
					'LGRPID LONG LONGLONG LONG_PTR LONG32 LONG64 LPARAM LPBOOL LPBYTE LPCOLORREF ' +
					'LPCSTR LPCTSTR LPCVOID LPCWSTR LPDWORD LPHANDLE LPINT LPLONG LPSTR LPTSTR ' +
					'LPVOID LPWORD LPWSTR LRESULT PBOOL PBOOLEAN PBYTE PCHAR PCSTR PCTSTR PCWSTR ' +
					'PDWORDLONG PDWORD_PTR PDWORD32 PDWORD64 PFLOAT PHALF_PTR PHANDLE PHKEY PINT ' +
					'PINT_PTR PINT32 PINT64 PLCID PLONG PLONGLONG PLONG_PTR PLONG32 PLONG64 POINTER_32 ' +
					'POINTER_64 PSHORT PSIZE_T PSSIZE_T PSTR PTBYTE PTCHAR PTSTR PUCHAR PUHALF_PTR ' +
					'PUINT PUINT_PTR PUINT32 PUINT64 PULONG PULONGLONG PULONG_PTR PULONG32 PULONG64 ' +
					'PUSHORT PVOID PWCHAR PWORD PWSTR SC_HANDLE SC_LOCK SERVICE_STATUS_HANDLE SHORT ' +
					'SIZE_T SSIZE_T TBYTE TCHAR UCHAR UHALF_PTR UINT UINT_PTR UINT32 UINT64 ULONG ' +
					'ULONGLONG ULONG_PTR ULONG32 ULONG64 USHORT USN VOID WCHAR WORD WPARAM WPARAM WPARAM ' +
					'char bool short int __int32 __int64 __int8 __int16 long float double __wchar_t ' +
					'clock_t _complex _dev_t _diskfree_t div_t ldiv_t _exception _EXCEPTION_POINTERS ' +
					'FILE _finddata_t _finddatai64_t _wfinddata_t _wfinddatai64_t __finddata64_t ' +
					'__wfinddata64_t _FPIEEE_RECORD fpos_t _HEAPINFO _HFILE lconv intptr_t ' +
					'jmp_buf mbstate_t _off_t _onexit_t _PNH ptrdiff_t _purecall_handler ' +
					'sig_atomic_t size_t _stat __stat64 _stati64 terminate_function ' +
					'time_t __time64_t _timeb __timeb64 tm uintptr_t _utimbuf ' +
					'va_list wchar_t wctrans_t wctype_t wint_t signed unsigned';

	var keywords =	'break case catch class const __finally __exception __try ' +
					'const_cast continue private public protected __declspec ' +
					'default delete deprecated dllexport dllimport do dynamic_cast ' +
					'else enum explicit extern if for friend goto inline ' +
					'mutable naked namespace new noinline noreturn nothrow ' +
					'register reinterpret_cast return selectany ' +
					'sizeof static static_cast struct switch template this ' +
					'thread throw true false try typedef typeid typename union ' +
					'using uuid virtual void volatile whcar_t while';
					
	var functions =	'assert isalnum isalpha iscntrl isdigit isgraph islower isprint' +
					'ispunct isspace isupper isxdigit tolower toupper errno localeconv ' +
					'setlocale acos asin atan atan2 ceil cos cosh exp fabs floor fmod ' +
					'frexp ldexp log log10 modf pow sin sinh sqrt tan tanh jmp_buf ' +
					'longjmp setjmp raise signal sig_atomic_t va_arg va_end va_start ' +
					'clearerr fclose feof ferror fflush fgetc fgetpos fgets fopen ' +
					'fprintf fputc fputs fread freopen fscanf fseek fsetpos ftell ' +
					'fwrite getc getchar gets perror printf putc putchar puts remove ' +
					'rename rewind scanf setbuf setvbuf sprintf sscanf tmpfile tmpnam ' +
					'ungetc vfprintf vprintf vsprintf abort abs atexit atof atoi atol ' +
					'bsearch calloc div exit free getenv labs ldiv malloc mblen mbstowcs ' +
					'mbtowc qsort rand realloc srand strtod strtol strtoul system ' +
					'wcstombs wctomb memchr memcmp memcpy memmove memset strcat strchr ' +
					'strcmp strcoll strcpy strcspn strerror strlen strncat strncmp ' +
					'strncpy strpbrk strrchr strspn strstr strtok strxfrm asctime ' +
					'clock ctime difftime gmtime localtime mktime strftime time';
	
	var ojtypes =	'JausAddress JausComponent JausNode JausService JausState JausSubsystem ' +
					'JausMessage JausBoolean JausByte JausDouble JausEventLimit JausFloat ' +
					'JausGeometryPointLLA JausGeometryPointXYZ JausInteger JausLong JausMissionCommand ' +
					'JausMissionTask JausShort JausTime JausUnsignedInteger JausUnsignedLong JausUnsignedShort ' +
					'JausWorldModelFeatureClass JausWorldModelVectorObject OjCmpt ReportGlobalPoseMessage ' +
					'QueryGlobalPoseMessage';

	var ojfuncs =	'reportGlobalPoseMessageCreate queryGlobalPoseMessageDestroy ojCmptSendMessage jausMessageDestroy ' +
					'queryGlobalPoseMessageFromJausMessage reportGlobalPoseMessageDestroy jausAddressCopy ojCmptSetMessageCallback ' +
					'jausUnsignedShortSetBit ojCmptAddService ojCmptAddServiceInputMessage ojCmptAddServiceOutputMessage ' +
					'ojCmptSetStateCallback ojCmptSetState ojCmptRun ojCmptDestroy ojCmptEstablishSc ojCmptTerminateSc ' +
					'ojCmptAddSupportedSc ojCmptIsOutgoingScActive ojCmptGetScSendList ojCmptDestroySendList ojCmptRemoveSupportedSc ' +
					'ojCmptGetAuthority jausAddressEqual'; 

	var ojdefines = 'JAUS_EXPORT JAUS_UNDEFINED_STATE JAUS_UNKNOWN_STATE JAUS_INVALID_STATE JAUS_PI JAUS_TRUE JAUS_FALSE ' + 
					'JAUS_INITIALIZE_STATE JAUS_READY_STATE JAUS_STANDBY_STATE ' +
					'JAUS_SHUTDOWN_STATE JAUS_FAILURE_STATE JAUS_EMERGENCY_STATE ' +
					'JAUS_NODE_MANAGER JAUS_SUBSYSTEM_COMMANDER JAUS_PRIMITIVE_DRIVER ' +
					'JAUS_GLOBAL_VECTOR_DRIVER JAUS_COMMUNICATOR JAUS_MISSION_SPOOLER ' +
					'JAUS_VISUAL_SENSOR JAUS_GLOBAL_POSE_SENSOR JAUS_MISSION_PLANNER ' +
					'JAUS_SYSTEM_COMMANDER JAUS_LOCAL_POSE_SENSOR JAUS_VELOCITY_STATE_SENSOR ' +
					'JAUS_REFLEXIVE_DRIVER JAUS_LOCAL_VECTOR_DRIVER JAUS_GLOBAL_WAYPOINT_DRIVER ' +
					'JAUS_LOCAL_WAYPOINT_DRIVER JAUS_GLOBAL_PATH_SEGMENT_DRIVER ' + 
					'JAUS_LOCAL_PATH_SEGMENT_DRIVER JAUS_PRIMITIVE_MANIPULATOR JAUS_RANGE_SENSOR ' +
					'JAUS_MANIPULATOR_JOINT_POSITION_SENSOR JAUS_MANIPULATOR_JOINT_VELOCITY_SENSOR ' +
					'JAUS_MANIPULATOR_JOINT_FORCE_TORQUE_SENSOR JAUS_MANIPULATOR_JOINT_POSITIONS_DRIVER ' +
					'JAUS_MANIPULATOR_END_EFFECTOR_DRIVER JAUS_MANIPULATOR_JOINT_VELOCITIES_DRIVER ' +
					'JAUS_MANIPULATOR_END_EFFECTOR_VELOCITY_STATE_DRIVER JAUS_MANIPULATOR_JOINT_MOVE_DRIVER ' +
					'JAUS_MANIPULATOR_END_EFFECTOR_DISCRETE_POSE_DRIVER JAUS_WORLD_MODEL_VECTOR_KNOWLEDGE_STORE ' +
					'JAUS_HEADER_SIZE_BYTES JAUS_LOW_PRIORITY JAUS_DEFAULT_PRIORITY JAUS_HIGH_PRIORITY ' +
					'JAUS_ACK_NAK_NOT_REQUIRED JAUS_ACK_NAK_REQUIRED JAUS_NEGATIVE_ACKNOWLEDGE JAUS_ACKNOWLEDGE ' +
					'JAUS_ACK JAUS_NAK JAUS_SERVICE_CONNECTION_MESSAGE JAUS_NOT_SERVICE_CONNECTION_MESSAGE ' +
					'JAUS_EXPERIMENTAL_MESSAGE JAUS_NOT_EXPERIMENTAL_MESSAGE JAUS_VERSION_2_0 JAUS_VERSION_2_1 ' +
					'JAUS_VERSION_3_0 JAUS_VERSION_3_1 JAUS_VERSION_3_2 JAUS_VERSION_3_3 JAUS_MAX_DATA_SIZE_BYTES ' +
					'JAUS_SINGLE_DATA_PACKET JAUS_FIRST_DATA_PACKET JAUS_NORMAL_DATA_PACKET JAUS_RETRANSMITTED_DATA_PACKET ' +
					'JAUS_LAST_DATA_PACKET JAUS_MAX_SEQUENCE_NUMBER JAUS_SET_COMPONENT_AUTHORITY JAUS_SHUTDOWN ' +
					'JAUS_STANDBY JAUS_RESUME JAUS_RESET JAUS_SET_EMERGENCY JAUS_CLEAR_EMERGENCY ' +
					'JAUS_CREATE_SERVICE_CONNECTION JAUS_CONFIRM_SERVICE_CONNECTION JAUS_ACTIVATE_SERVICE_CONNECTION ' +
					'JAUS_SUSPEND_SERVICE_CONNECTION JAUS_TERMINATE_SERVICE_CONNECTION JAUS_REQUEST_COMPONENT_CONTROL ' +
					'JAUS_RELEASE_COMPONENT_CONTROL JAUS_CONFIRM_COMPONENT_CONTROL JAUS_REJECT_COMPONENT_CONTROL ' +
					'JAUS_SET_TIME JAUS_CREATE_EVENT JAUS_UPDATE_EVENT JAUS_CANCEL_EVENT JAUS_CONFIRM_EVENT_REQUEST ' +
					'JAUS_REJECT_EVENT_REQUEST JAUS_SET_DATA_LINK_STATUS JAUS_SET_DATA_LINK_SELECT ' +
					'JAUS_SET_SELECTED_DATA_LINK_STATE JAUS_SET_WRENCH_EFFORT JAUS_SET_DISCRETE_DEVICES ' +
					'JAUS_SET_GLOBAL_VECTOR JAUS_SET_LOCAL_VECTOR JAUS_SET_TRAVEL_SPEED JAUS_SET_GLOBAL_WAYPOINT ' +
					'JAUS_SET_LOCAL_WAYPOINT JAUS_SET_GLOBAL_PATH_SEGMENT JAUS_SET_LOCAL_PATH_SEGMENT JAUS_SET_JOINT_EFFORTS ' +
					'JAUS_SET_JOINT_POSITIONS JAUS_SET_JOINT_VELOCITIES JAUS_SET_TOOL_POINT JAUS_SET_END_EFFECTOR_POSE ' +
					'JAUS_SET_END_EFFECTOR_VELOCITY_STATE JAUS_SET_JOINT_MOTION JAUS_SET_END_EFFECTOR_PATH_MOTION ' +
					'JAUS_SET_CAMERA_POSE JAUS_SELECT_CAMERA JAUS_SET_CAMERA_CAPABILITIES JAUS_SET_CAMERA_FORMAT_OPTIONS ' +
					'JAUS_CREATE_VKS_OBJECTS JAUS_DELETE_VKS_OBJECTS JAUS_SET_VKS_FEATURE_CLASS_METADATA ' +
					'JAUS_TERMINATE_VKS_DATA_TRANSFER JAUS_SET_PAYLOAD_DATA_ELEMENT JAUS_SPOOL_MISSION JAUS_RUN_MISSION ' +
					'JAUS_ABORT_MISSION JAUS_PAUSE_MISSION JAUS_RESUME_MISSION JAUS_REMOVE_MESSAGES JAUS_REPLACE_MESSAGES ' +
					'JAUS_QUERY_COMPONENT_AUTHORITY JAUS_QUERY_COMPONENT_STATUS JAUS_QUERY_TIME JAUS_QUERY_COMPONENT_CONTROL ' +
					'JAUS_QUERY_EVENTS JAUS_QUERY_DATA_LINK_STATUS JAUS_QUERY_SELECTED_DATA_LINK_STATUS JAUS_QUERY_HEARTBEAT_PULSE ' +
					'JAUS_QUERY_PLATFORM_SPECIFICATIONS JAUS_QUERY_PLATFORM_OPERATIONAL_DATA JAUS_QUERY_GLOBAL_POSE ' +
					'JAUS_QUERY_LOCAL_POSE JAUS_QUERY_VELOCITY_STATE JAUS_QUERY_WRENCH_EFFORT JAUS_QUERY_DISCRETE_DEVICES ' +
					'JAUS_QUERY_GLOBAL_VECTOR JAUS_QUERY_LOCAL_VECTOR JAUS_QUERY_TRAVEL_SPEED JAUS_QUERY_WAYPOINT_COUNT ' +
					'JAUS_QUERY_GLOBAL_WAYPOINT JAUS_QUERY_LOCAL_WAYPOINT JAUS_QUERY_PATH_SEGMENT_COUNT ' +
					'JAUS_QUERY_GLOBAL_PATH_SEGMENT JAUS_QUERY_LOCAL_PATH_SEGMENT JAUS_QUERY_MANIPULATOR_SPECIFICATIONS ' +
					'JAUS_QUERY_JOINT_EFFORTS JAUS_QUERY_JOINT_POSITIONS JAUS_QUERY_JOINT_VELOCITIES JAUS_QUERY_TOOL_POINT ' +
					'JAUS_QUERY_JOINT_FORCE_TORQUES JAUS_QUERY_CAMERA_POSE JAUS_QUERY_CAMERA_COUNT ' +
					'JAUS_QUERY_RELATIVE_OBJECT_POSITION JAUS_QUERY_SELECTED_CAMERA JAUS_QUERY_CAMERA_CAPABILITIES ' +
					'JAUS_QUERY_CAMERA_FORMAT_OPTIONS JAUS_QUERY_IMAGE JAUS_QUERY_VKS_FEATURE_CLASS_METADATA ' +
					'JAUS_QUERY_VKS_BOUNDS JAUS_QUERY_VKS_OBJECTS JAUS_QUERY_IDENTIFICATION JAUS_QUERY_CONFIGURATION ' +
					'JAUS_QUERY_SUBSYSTEM_LIST JAUS_QUERY_SERVICES JAUS_QUERY_PAYLOAD_INTERFACE JAUS_QUERY_PAYLOAD_DATA_ELEMENT ' +
					'JAUS_QUERY_SPOOLING_PREFERENCE JAUS_QUERY_MISSION_STATUS JAUS_POSE_PV_LATITUDE_BIT JAUS_POSE_PV_LONGITUDE_BIT ' +
					'JAUS_REPORT_GLOBAL_POSE';

	this.regexList = [
		{ regex: SyntaxHighlighter.regexLib.singleLineCComments,	css: 'comments' },			// one line comments
		{ regex: SyntaxHighlighter.regexLib.multiLineCComments,		css: 'comments' },			// multiline comments
		{ regex: SyntaxHighlighter.regexLib.doubleQuotedString,		css: 'string' },			// strings
		{ regex: SyntaxHighlighter.regexLib.singleQuotedString,		css: 'string' },			// strings
		{ regex: /^ *#.*/gm,										css: 'preprocessor' },
		{ regex: new RegExp(this.getKeywords(datatypes), 'gm'),		css: 'color1 bold' },
		{ regex: new RegExp(this.getKeywords(ojtypes), 'gm'),		css: 'color1 bold' },		// OpenJAUS Datatypes
		{ regex: new RegExp(this.getKeywords(functions), 'gm'),		css: 'functions bold' },
		{ regex: new RegExp(this.getKeywords(ojfuncs), 'gm'),		css: 'functions bold' },	// OpenJAUS Functions
		{ regex: new RegExp(this.getKeywords(keywords), 'gm'),		css: 'keyword bold' },
		{ regex: new RegExp(this.getKeywords(ojdefines), 'gm'),		css: 'preprocessor bold' }	// OpenJAUS #defines
		];
};

SyntaxHighlighter.brushes.Oj.prototype	= new SyntaxHighlighter.Highlighter();
SyntaxHighlighter.brushes.Oj.aliases	= ['ojcpp', 'oj'];
