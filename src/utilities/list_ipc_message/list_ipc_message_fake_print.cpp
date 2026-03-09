#define __CYGWIN__
#define REDHAT_52 
#define REDHAT_6 
#define REDHAT_71

#ifdef __cplusplus
extern "C" {
#endif

#include "globalS.h"

#ifdef __cplusplus
}
#endif

#undef PRINT_LENGTH
#define PRINT_LENGTH 100

/* Defined below; forward reference */
static void FAKE_Print_Formatter1(FILE *stream, Print_Data_Ptr printer,
			     CONST_FORMAT_PTR format, int32 keepWithNext);

static void FAKE_Print_Structured_Data (FILE *Stream, Print_Data_Ptr printer,
				   CONST_FORMAT_PTR Format,
				   CONST_GENERIC_DATA_PTR Data_Ptr, int32 DStart,
				   CONST_FORMAT_PTR parentFormat, 
				   int32 Keep_With_Next);

// Codigo para printar campos, conforme vai entrando na mensagem, trunca antes os campos
#define FIELD_LEVEL1_SIZE 50 // Comparavel a PRINT_LENGTH // Tamanho da mensagem?
#define FIELD_LEVEL2_SIZE 4 // Comparavel a PRINT_LENGTH
#define FIELD_LEVEL3_SIZE 4 // Comparavel a PRINT_LENGTH
#define FIELD_LEVEL4_SIZE 4 // Comparavel a PRINT_LENGTH 
#define FIELD_LEVEL5_SIZE 4 // Comparavel a PRINT_LENGTH 
#define FIELD_LEVEL6_SIZE 4 // Comparavel a PRINT_LENGTH 
static int field_level1_num_lines = 0; // Quantidade de linhas printadas no campo primário
static int field_level2_num_lines = 0; // Quantidade de linhas printadas no campo secundário
static int field_level3_num_lines = 0; // Quantidade de linhas printadas no campo terciário
static int field_level4_num_lines = 0; // Quantidade de linhas printadas no campo quartenário
static int field_level5_num_lines = 0; // Quantidade de linhas printadas 
static int field_level6_num_lines = 0; // Quantidade de linhas printadas 

static int present_field_level = 0; // Nivel atual que está
static int truncate_field = 0;


/******************************************************************************
 *t
 * FUNCTION: void is_field_size_exceeded
 *
 ****/

static int is_field_size_exceeded()
{
  switch (present_field_level)
  {
  case 0:
    return 0;
  case 1:
    return (field_level1_num_lines > FIELD_LEVEL1_SIZE);
  case 2:
    return (field_level2_num_lines > FIELD_LEVEL2_SIZE);
  case 3:
    return (field_level3_num_lines > FIELD_LEVEL3_SIZE);
  case 4:
    return (field_level4_num_lines > FIELD_LEVEL4_SIZE);
  case 5:
    return (field_level5_num_lines > FIELD_LEVEL5_SIZE);
  case 6:
    return (field_level6_num_lines > FIELD_LEVEL6_SIZE);
  default:
    return (field_level6_num_lines > FIELD_LEVEL6_SIZE);
  }
  return 0;
}



/******************************************************************************
 *t
 * FUNCTION: void return_field_level
 *
 *****************************************************************************/

static void return_field_level()
{
  switch (present_field_level)
  {
  case 1:
    field_level1_num_lines = 0;
    break;
  case 2:
    field_level2_num_lines = 0;
    break;
  case 3:
    field_level3_num_lines = 0;
    break;
  case 4:
    field_level4_num_lines = 0;
    break;
  case 5:
    field_level5_num_lines = 0;
    break;
  case 6:
    field_level6_num_lines = 0;
    break;
  default:
    break;
  }
  present_field_level--;
  truncate_field = 0;
}


/******************************************************************************
 *t
 * FUNCTION: void advance_field_level
 *
 *****************************************************************************/

static void advance_field_level()
{
  switch (present_field_level)
  {
  case 0:
    field_level1_num_lines = 0;
    break;
  case 1:
    field_level2_num_lines = 0;
    break;
  case 2:
    field_level3_num_lines = 0;
    break;
  case 3:
    field_level4_num_lines = 0;
    break;
  case 4:
    field_level5_num_lines = 0;
    break;
  case 5:
    field_level6_num_lines = 0;
    break;
  default:
    break;
  }
  present_field_level++;
}


/******************************************************************************
 *t
 * FUNCTION: void increase_field_line_count
 *
 *****************************************************************************/

static void increase_field_line_count()
{
  switch (present_field_level)
  {
  case 1:
    field_level1_num_lines += 1;
    break;
  case 2:
    field_level1_num_lines += 1;
    field_level2_num_lines += 1;
    break;
  case 3:
    field_level1_num_lines += 1;
    field_level2_num_lines += 1;
    field_level3_num_lines += 1;
    break;
  case 4:
    field_level1_num_lines += 1;
    field_level2_num_lines += 1;
    field_level3_num_lines += 1;
    field_level4_num_lines += 1;
    break;
  case 5:
    field_level1_num_lines += 1;
    field_level2_num_lines += 1;
    field_level3_num_lines += 1;
    field_level4_num_lines += 1;
    field_level5_num_lines += 1;
    break;
  case 6:
    field_level1_num_lines += 1;
    field_level2_num_lines += 1;
    field_level3_num_lines += 1;
    field_level4_num_lines += 1;
    field_level5_num_lines += 1;
    field_level6_num_lines += 1;
    break;
  default:
    field_level1_num_lines += 1;
    field_level2_num_lines += 1;
    field_level3_num_lines += 1;
    field_level4_num_lines += 1;
    field_level5_num_lines += 1;
    field_level6_num_lines += 1;
    break;
  }
}


/******************************************************************************
 *
 * FUNCTION: void FAKE_newLine(stream)
 *
 * DESCRIPTION:
 *
 * INPUTS: FILE *stream;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

static void FAKE_newLine(FILE *stream, Print_Data_Ptr printer)
{
  (void)fprintf(stream, "\n");
  printer->lineNumGlobal++;
  increase_field_line_count();
  printer->cursorPosGlobal = 0;
}


/******************************************************************************
 *
 * FUNCTION: void FAKE_printTab(stream, tabPosition)
 *
 * DESCRIPTION: 
 * Print spaces from current cursorPosGlobal to Tab_Position.
 * Does nothing if position already passed.
 *
 * INPUTS: 
 * FILE *stream;
 * int tabPosition;
 *
 * OUTPUTS: void.
 *
 * NOTES: makes use of printf min field width.
 *
 *****************************************************************************/

static void FAKE_printTab(FILE *stream, Print_Data_Ptr printer, int32 tabPosition)
{ 
  int32 spaces;
  
  if (!printer->truncatedGlobal) {
    if (printer->cursorPosGlobal < tabPosition) {
      spaces = tabPosition - printer->cursorPosGlobal;
      (void)fprintf(stream, "%*s", spaces, " ");
      printer->cursorPosGlobal += spaces;
    }
  }
}


/******************************************************************************
 *
 * FUNCTION: void FAKE_printSpace(stream)
 *
 * DESCRIPTION:
 * Print out a space to the stream, unless at the end of a line,
 * in which case, go to the next line.
 *
 * INPUTS: 
 *
 * OUTPUTS: 
 *
 *****************************************************************************/

static void FAKE_printSpace(FILE *stream, Print_Data_Ptr printer)
{
  if (!printer->truncatedGlobal) {
    if (printer->cursorPosGlobal == LINE_LENGTH) {
      FAKE_newLine(stream, printer);
      FAKE_printTab(stream, printer, GET_M_GLOBAL(indentGlobal));
    }
    
    (void)fprintf(stream, "%c", ' ');
    printer->cursorPosGlobal++;
  }
}


/******************************************************************************
 *
 * FUNCTION: void FAKE_printString(stream, string, keepWithNext)
 *
 * DESCRIPTION:
 * Print out the string to the stream.
 * Move to next line if less than "keep_with_next" characters would
 * remain on the current line after printing the string.
 *
 * INPUTS: 
 * FILE *stream;
 * char *string;
 * int32 keepWithNext;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

static void FAKE_printString(FILE *stream, Print_Data_Ptr printer,
		 const char *pString, int32 keepWithNext)
{ 
  int32 length;
  
  if (!printer->truncatedGlobal) {
    if (pString)
      length = strlen(pString);
    else
      length = 0;
    
    if ((printer->cursorPosGlobal + length + keepWithNext) > LINE_LENGTH) 
      {
	FAKE_newLine(stream, printer);
	FAKE_printTab(stream, printer, GET_M_GLOBAL(indentGlobal)+1);
      }
    if (is_field_size_exceeded())
    {
      truncate_field = 1;
    }
    if (printer->lineNumGlobal < PRINT_LENGTH) {
      if (pString) {
	(void)fprintf(stream, "%s", pString);
	printer->cursorPosGlobal += length;
      }
      else {
	(void)fprintf(stream, "NULL");
	printer->cursorPosGlobal += 4;
      }
    }
    else {
      printf("truncating...\n");
      printer->truncatedGlobal = 1;
      (void)fprintf(stream, "...");
    }
  }
}


/******************************************************************************
 *
 * FUNCTION: void FAKE_printInt(stream, intPtr, keepWithNext)
 *
 * DESCRIPTION:
 * Print out the integer to the stream.
 * Move to next line if less than "keep_with_next" characters would
 * remain on the current line after printing the integer.
 *
 * INPUTS: 
 * FILE *stream;
 * int *intPtr;
 * int32 keepWithNext;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

static void FAKE_printInt(FILE *stream, Print_Data_Ptr printer,
	      const int32 *intPtr, int32 keepWithNext)
{ 
  (void)sprintf(printer->buffer, "%d", *intPtr);
  FAKE_printString(stream, printer, printer->buffer, keepWithNext);
}


/******************************************************************************
 *
 * FUNCTION: void FAKE_printHex(stream, intPtr, keepWithNext)
 *
 * DESCRIPTION:
 *
 * INPUTS: 
 * FILE *stream;
 * unsigned *intPtr;
 * int KeepWithNext;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

static void FAKE_printHex(FILE *stream, Print_Data_Ptr printer,
		     const u_int32 *intPtr, int32 keepWithNext)
{
  (void)sprintf(printer->buffer, "%#x", *intPtr);
  FAKE_printString(stream, printer, printer->buffer, keepWithNext);
}


/******************************************************************************
 *
 * FUNCTION: void FAKE_printCommaSpace(stream)
 *
 * DESCRIPTION:
 *
 * INPUTS: FILE *stream;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

static void FAKE_printCommaSpace(FILE *stream, Print_Data_Ptr printer)
{
  FAKE_printString(stream, printer, ",", 0);
  FAKE_printSpace(stream, printer);
}

/******************************************************************************
 *
 * FUNCTION: void FAKE_printChar(stream, charPtr, keepWithNext)
 *
 * DESCRIPTION:
 *
 * INPUTS: 
 * FILE *stream;
 * char *charPtr;
 * int keepWithNext;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

void FAKE_printChar(FILE *stream, Print_Data_Ptr printer,
	       const char *charPtr, int32 keepWithNext)
{ 
  (void)sprintf(printer->buffer, "%c", *charPtr);
  FAKE_printString(stream, printer, printer->buffer, keepWithNext);
}

/******************************************************************************
 *
 * FUNCTION: void FAKE_printDouble(stream, doublePtr, keepWithNext)
 *
 * DESCRIPTION:
 * Print out the double floating_point number to the stream, with
 * accuracy of "DEC_PLACES".
 * Move to next line if less than "keep_with_next" characters would
 * remain on the current line after printing the double.
 *
 * INPUTS: 
 * FILE *stream;
 * double *doublePtr;
 * int32 keepWithNext;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

void FAKE_printDouble(FILE *stream, Print_Data_Ptr printer,
		 const double *doublePtr, int32 keepWithNext)
{ 
  (void)sprintf(printer->buffer, "%.*f", DEC_PLACES, *doublePtr);
  FAKE_printString(stream, printer, printer->buffer, keepWithNext);
}

void FAKE_dPrintSTR(FILE *stream, Print_Data_Ptr printer, 
	       const char *pString, int32 next)
{
  FAKE_printString(stream, printer, "\"", next);
  FAKE_printString(stream, printer, pString, next);
  FAKE_printString(stream, printer, "\"", next);
}

/******************************************************************************
 *
 * FUNCTION: void FAKE_printByte(stream, bytePtr, keepWithNext)
 *
 * DESCRIPTION:
 * Print out the byte (in hex) to the stream.
 * Move to next line if less than "keep_with_next" characters would
 * remain on the current line after printing the byte.
 *
 * INPUTS: 
 * FILE *stream;
 * char *bytePtr;
 * int keepWithNext;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

void FAKE_printByte(FILE *stream, Print_Data_Ptr printer,
	       const char *bytePtr, int32 keepWithNext)
{ 
  u_int32 byte =0;
  
  *(((char *)&byte)+sizeof(u_int32)-sizeof(char)) = *bytePtr;
  FAKE_printHex(stream, printer, &byte, keepWithNext);

  
}

/******************************************************************************
 *
 * FUNCTION: void FAKE_printShort(stream, shortPtr, keepWithNext)
 *
 * DESCRIPTION:
 *
 * INPUTS: 
 * FILE *stream;
 * short *shortPtr;
 * int keepWithNext;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

void FAKE_printShort(FILE *stream, Print_Data_Ptr printer,
		const int16 *shortPtr, int32 keepWithNext)
{ 
  (void)sprintf(printer->buffer, "%d", *shortPtr);
  FAKE_printString(stream, printer, printer->buffer, keepWithNext);
}


/******************************************************************************
 *
 * FUNCTION: void FAKE_startPrint(stream)
 *
 * DESCRIPTION:
 *
 * INPUTS: FILE *stream;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

static void FAKE_startPrint(FILE *stream, Print_Data_Ptr printer)
{
  printer->cursorPosGlobal = 0;
  printer->lineNumGlobal = 0;
  printer->truncatedGlobal = 0;
  truncate_field = 0;
  field_level1_num_lines = 0;
  field_level2_num_lines = 0;
  field_level3_num_lines = 0;
  field_level4_num_lines = 0;
  field_level5_num_lines = 0;
  field_level6_num_lines = 0;
  FAKE_printTab(stream, printer, GET_M_GLOBAL(indentGlobal));
}


/******************************************************************************
 *
 * FUNCTION: void FAKE_endPrint(stream)
 *
 * DESCRIPTION:
 *
 * INPUTS: FILE *stream;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

static void FAKE_endPrint(FILE *stream, Print_Data_Ptr printer)
{
  FAKE_newLine(stream, printer);
  printer->truncatedGlobal = 0;
  truncate_field = 0;
}


/******************************************************************************
 *
 * FUNCTION: int32 FAKE_printArrayData(stream, format, dataArray, AStart, 
 *                  formatArray, dimension, keepWithNext, startToken, endToken)
 *
 * DESCRIPTION:
 *
 * INPUTS: 
 * FILE *stream;
 * FORMAT_PTR format;
 * char *dataArray;
 * int32 AStart, dimension, keepWithNext;
 * FORMAT_ARRAY_PTR formatArray;
 * char *startToken, *endToken;
 *
 * OUTPUTS: int32
 *
 *****************************************************************************/

static int32 FAKE_printArrayData(FILE *stream, Print_Data_Ptr printer,
			    CONST_FORMAT_PTR format, 
			    const void *dataArray,
			    int32 AStart, FORMAT_ARRAY_PTR formatArray,
			    int32 dimension, int32 keepWithNext,
			    char *startToken, char *endToken)
{ 
  int32 i, element_size=0, last_dimension_p, last_element_p;
  int32 vector_length, next_keep;
  
  FAKE_printString(stream, printer, startToken, DEFAULT_KEEP);
  
  last_dimension_p = (dimension+1 == formatArray[0].i);
  if (last_dimension_p)
    element_size = x_ipc_dataStructureSize(format);
  
  vector_length = formatArray[dimension].i;
  
  for (i=0; i<vector_length; i++) {
    if (printer->truncatedGlobal || truncate_field) {
      FAKE_printString(stream, printer, endToken, DEFAULT_KEEP); 
      return 0;
    } else {
      last_element_p = (i+1 == vector_length);
      next_keep = ((last_element_p) ? keepWithNext+1 : 1);
      if (last_dimension_p) {
	FAKE_Print_Structured_Data(stream, printer, format,
			      (CONST_GENERIC_DATA_PTR)dataArray,
			      AStart, (FORMAT_PTR)NULL, next_keep);
	AStart += element_size;
	if (!last_element_p)
	  FAKE_printCommaSpace(stream, printer);
      }
      else {
	AStart = FAKE_printArrayData(stream, printer, format, dataArray, AStart, 
				formatArray, dimension+1, next_keep, 
				startToken, endToken);
	if (!last_element_p)
	  FAKE_printCommaSpace(stream, printer);
      }
    }
  }
  
  FAKE_printString(stream, printer, endToken, keepWithNext);
  
  return AStart;
}


/******************************************************************************
 *
 * FUNCTION: FORMAT_ARRAY_PTR FAKE_fix_format_array(varFormatArray, 
 *                           parentStructArray, dataPtr, DStart)
 *
 * DESCRIPTION:
 * Create a fixed format array that contains all the dimensions of
 * the variable length array.
 *
 * INPUTS: 
 * FORMAT_ARRAY_PTR varFormatArray, parentStructArray;
 * GENERIC_DATA_PTR dataPtr;
 * int32 DStart;
 *
 * OUTPUTS: 
 *
 *****************************************************************************/

/* RTG: The method of finding the size of the variable array was not quite 
 * correct.  Because of the way padding can be added before the pointer
 * to the variable array, just backing up or going forward the minimum
 * number of bytes is not always valid.  This is more apparent when running
 * on an alpha because pointers and ints are different sizes.
 */

static FORMAT_ARRAY_PTR FAKE_fix_format_array(FORMAT_ARRAY_PTR varFormatArray, 
					 CONST_FORMAT_PTR parentFormat,
					 CONST_GENERIC_DATA_PTR dataPtr,
					 int32 DStart)
{ 
  FORMAT_ARRAY_PTR fixed_format_array;
  FORMAT_ARRAY_PTR parentStructArray;
  int32 arraySize, foundPlace, currentPlace, i, j, sizePlace;
  int32 offset=0, size, sizeOffset=0, currentOffset=0;
  
  SIZES_TYPE sizes;
  
  parentStructArray = parentFormat->formatter.a;
  foundPlace = 0;
  for (currentPlace=1; !foundPlace; currentPlace++)
    foundPlace = ((parentStructArray[currentPlace].f->type == VarArrayFMT) &&
		  (parentStructArray[currentPlace].f->formatter.a
		   == varFormatArray));
  currentPlace--;
  
  arraySize = varFormatArray[0].i;
  fixed_format_array = (FORMAT_ARRAY_PTR)x_ipcMalloc((unsigned)arraySize * 
						   sizeof(FORMAT_ARRAY_TYPE));
  fixed_format_array[0].i = arraySize;
  fixed_format_array[1].f = varFormatArray[1].f;
  for (i=2; i<arraySize; i++) {
    sizePlace = varFormatArray[i].i;
    offset = 0;
    sizeOffset = 0;
    j=1;
    while((j <= sizePlace) || (j <= currentPlace)) {
      switch (parentStructArray[j].f->type) {
      case VarArrayFMT:
	sizes.data = sizeof(GENERIC_DATA_PTR);
	break;
      default:
	sizes = x_ipc_bufferSize1(parentStructArray[j].f,dataPtr,
			    sizeOffset,parentFormat);
	break;
      }
      offset += sizes.data;
      offset = x_ipc_alignField(parentFormat,j,offset);
      j++;
      if (j <= sizePlace) {
	sizeOffset = offset;
      }
      if (j <= currentPlace) {
	currentOffset = offset;
      }
    }
    
    offset = sizeOffset - currentOffset;
    size = *((int32 *)(((char *)dataPtr)+DStart+offset));
    fixed_format_array[i].i = size;
  }
  
  return fixed_format_array;
}

// typedef enum {
//   INT_FMT =     1,
//   BOOLEAN_FMT = 2,
//   FLOAT_FMT =   3,
//   DOUBLE_FMT =  4,
//   BYTE_FMT =    5,
//   TWOBYTE_FMT = 6,
//   STR_FMT =     7,
//   FORMAT_FMT =  8,
//   UBYTE_FMT =   9,
//   CMAT_FMT =    10,
//   SMAT_FMT =    11,
//   IMAT_FMT =    12,
//   LMAT_FMT =    13,
//   FMAT_FMT =    14,
//   DMAT_FMT =    15,
//   CHAR_FMT =    16,
//   SHORT_FMT =   17,
//   LONG_FMT =    18,
//   UCMAT_FMT =   19,
//   X_IPC_REF_PTR_FMT = 20,
  
//   SIUCMAT_FMT = 21,
//   SICMAT_FMT =  22,
//   SISMAT_FMT =  23,
//   SIIMAT_FMT =  24,
//   SILMAT_FMT =  25,
//   SIFMAT_FMT =  26,
//   SIDMAT_FMT =  27,

//   USHORT_FMT =  28,
//   UINT_FMT =    29,
//   ULONG_FMT =   30,

//   MAXFORMATTERS = 31
// #ifdef FORCE_32BIT_ENUM
//     , dummyPrimFormat = 0x7FFFFFFF
// #endif
// } PRIM_FORMAT_TYPE;

// static void x_ipc_printDataModuleInitialize(void)
// {
//   LOCK_M_MUTEX;
//   GET_M_GLOBAL(byteFormat) = ParseFormatString("byte");
//   GET_M_GLOBAL(charFormat) = ParseFormatString("char");
//   GET_M_GLOBAL(shortFormat) = ParseFormatString("short");
//   GET_M_GLOBAL(intFormat) = ParseFormatString("int");
//   GET_M_GLOBAL(longFormat) = ParseFormatString("long");
//   GET_M_GLOBAL(floatFormat) = ParseFormatString("float");
//   GET_M_GLOBAL(doubleFormat) = ParseFormatString("double");
  
//   GET_M_GLOBAL(dPrintSTR_FN) = dPrintSTR;
//   GET_M_GLOBAL(dPrintFORMAT_FN) = dPrintFORMAT;
// #ifndef NMP_IPC
//   GET_M_GLOBAL(dPrintMAP_FN) = mapPrint;
// #endif
//   GET_M_GLOBAL(dPrintX_IPC_FN) = dPrintX_IPC;
//   GET_M_GLOBAL(dPrintCHAR_FN) = printChar;
//   GET_M_GLOBAL(dPrintSHORT_FN) = printShort;
//   GET_M_GLOBAL(dPrintLONG_FN) = printLong;
//   GET_M_GLOBAL(dPrintINT_FN) = printInt;
//   GET_M_GLOBAL(dPrintBOOLEAN_FN) = printBoolean;
//   GET_M_GLOBAL(dPrintFLOAT_FN) = printFloat;
//   GET_M_GLOBAL(dPrintDOUBLE_FN) = printDouble;
//   GET_M_GLOBAL(dPrintBYTE_FN) = printByte;
//   GET_M_GLOBAL(dPrintUBYTE_FN) = printUByte;
//   GET_M_GLOBAL(dPrintTWOBYTE_FN) = printTwoByte;

//   GET_M_GLOBAL(dPrintUSHORT_FN) = printUShort;
//   GET_M_GLOBAL(dPrintUINT_FN) = printUInt;
//   GET_M_GLOBAL(dPrintULONG_FN) = printULong;
//   UNLOCK_M_MUTEX;
// }


/******************************************************************************
 *
 * FUNCTION:
 *
 * DESCRIPTION:
 * If there is no formatter, it means that the ptr is recursive (self-ptr)
 *
 * INPUTS: 
 *
 * OUTPUTS: 
 *
 *****************************************************************************/

static void FAKE_Print_Structured_Data(FILE *Stream, Print_Data_Ptr printer,
				  CONST_FORMAT_PTR Format,
				  CONST_GENERIC_DATA_PTR Data_Ptr, 
				  int32 DStart,
				  CONST_FORMAT_PTR parentFormat, 
				  int32 Keep_With_Next)
{ 
  TRANSLATE_FN_DPRINT printProc;
  int32 i, last_element_p, next_keep, length, currentDStart;
  u_int32 byte;
  void *StructPtr, *ArrayPtr;
  FORMAT_ARRAY_PTR format_array, fixed_format_array;

  currentDStart = DStart;
  switch (Format->type) {
  case LengthFMT:
    advance_field_level();
    length = Format->formatter.i;
    FAKE_printString(Stream, printer, "(", DEFAULT_KEEP);
    for (i=0; i<length; i++) {
      if (printer->truncatedGlobal || truncate_field) {
        FAKE_printString(Stream, printer, ")", Keep_With_Next);
        return_field_level();
	      return;
      } else {
	byte = 0;
	*(((u_char *)&byte)+sizeof(u_int32)-sizeof(u_char)) =
	  *(u_char *)(Data_Ptr+currentDStart+i);
	FAKE_printHex(Stream, printer, &byte,
		 (i == length-1 ? Keep_With_Next+1 : 1));
	if (i == length-1) FAKE_printString(Stream, printer, ")", Keep_With_Next);
	else FAKE_printCommaSpace(Stream, printer);
      }
    }
    return_field_level();
    break;
    
  case PrimitiveFMT: 
    advance_field_level();
    // printProc = GET_M_GLOBAL(TransTable)[Format->formatter.i].DPrint;

    

    switch (Format->formatter.i)
    {
    case 1: // int
      FAKE_printInt(Stream, printer, (const int32*) Data_Ptr, Keep_With_Next);
      break;
    case 4: // double
      FAKE_printDouble(Stream, printer, (const double*) Data_Ptr, Keep_With_Next);
      break;
    case 5: // byte
      FAKE_printByte(Stream, printer, (const char*) Data_Ptr, Keep_With_Next);
      break;
    case 7: // string
      FAKE_dPrintSTR(Stream, printer, REF(char *, Data_Ptr,currentDStart), Keep_With_Next);
      break;
    case 16: // char
      FAKE_printChar(Stream, printer, (const char*) Data_Ptr, Keep_With_Next);
      break;
    case 17: // short
      FAKE_printChar(Stream, printer, (const char*) Data_Ptr, Keep_With_Next);
      break;
    
    default:
      printf("numero: %d\n", Format->formatter.i);
    }
    // (* printProc)(Data_Ptr, currentDStart, Stream, printer, Keep_With_Next);
    return_field_level();
    break;
    
  case PointerFMT:
  advance_field_level();
    StructPtr = REF(GENERIC_DATA_PTR, Data_Ptr, currentDStart);
    if (StructPtr) { 
      FAKE_printString(Stream, printer, "*", DEFAULT_KEEP);
      FAKE_Print_Structured_Data(Stream, printer,
			    CHOOSE_PTR_FORMAT(Format, parentFormat),
			    (CONST_GENERIC_DATA_PTR)StructPtr, 0,
			    (CONST_FORMAT_PTR)NULL, Keep_With_Next);
    }
    else
      FAKE_printString(Stream, printer, "NULL", Keep_With_Next);
    return_field_level();
    break;
    
  case StructFMT:
    advance_field_level();
    FAKE_printString(Stream, printer, "{", DEFAULT_KEEP);
    format_array = Format->formatter.a;
    for (i=1;i<format_array[0].i;i++) {
      if (printer->truncatedGlobal || truncate_field) {
        FAKE_printString(Stream, printer, "}", Keep_With_Next);
        return_field_level();
	      return;
      } else {
	last_element_p = (i+1 == format_array[0].i);
	next_keep = ((last_element_p) ? Keep_With_Next+1 : 1);
	FAKE_Print_Structured_Data(Stream, printer, format_array[i].f, 
			      Data_Ptr+DStart, currentDStart-DStart, 
			      Format, next_keep);
	currentDStart += x_ipc_dataStructureSize(format_array[i].f);
	currentDStart = x_ipc_alignField(Format, i, currentDStart);
	if (last_element_p)
	  FAKE_printString(Stream, printer, "}", Keep_With_Next);
	else
	  FAKE_printCommaSpace(Stream, printer);
      }
    }
    return_field_level();
    break;
    
  case FixedArrayFMT:
    advance_field_level();
    (void)FAKE_printArrayData(Stream, printer, Format->formatter.a[1].f,
			 Data_Ptr, currentDStart, 
			 Format->formatter.a, 2, Keep_With_Next, (char*)"[", (char*)"]");
    return_field_level();
    break;
    
  case VarArrayFMT:
    advance_field_level();
    ArrayPtr = REF(GENERIC_DATA_PTR, Data_Ptr, currentDStart);
    format_array = Format->formatter.a;
    if (ArrayPtr) {
      fixed_format_array = FAKE_fix_format_array(format_array, 
					    parentFormat,
					    Data_Ptr, currentDStart);
      (void)FAKE_printArrayData(Stream, printer, format_array[1].f, ArrayPtr, 0, 
			   fixed_format_array, 2, Keep_With_Next, (char*)"<", (char*)">");
      x_ipcFree((char *)fixed_format_array);
    }
    else {
      FAKE_printString(Stream, printer, "NULL", Keep_With_Next);
    }
    return_field_level();
    break;
  case NamedFMT:
    advance_field_level();
    FAKE_Print_Structured_Data(Stream, printer,
			  x_ipc_fmtFind(Format->formatter.name),
			  Data_Ptr, currentDStart,
			  parentFormat, Keep_With_Next);
    return_field_level();
    break;
  case BadFormatFMT:
    advance_field_level();
    FAKE_printString(Stream, printer, "Bad Format", Keep_With_Next);
    return_field_level();
    break;
  case EnumFMT:
    { advance_field_level();
      int32 eVal;
      eVal = x_ipc_enumToInt(Format, Data_Ptr, &currentDStart);
      if (Format->formatter.a[0].i > 2 &&
	  /* enum value within range */
	  0 <= eVal && eVal <= ENUM_MAX_VAL(Format)) {
	FAKE_printString(Stream, printer,
		    Format->formatter.a[eVal+2].f->formatter.name,
		    Keep_With_Next);
      } else {
	FAKE_printInt(Stream, printer, &eVal, Keep_With_Next);
      }
      return_field_level();
      break;
    }

#ifndef TEST_CASE_COVERAGE
  default:
    advance_field_level();
    X_IPC_MOD_ERROR1("Unknown FAKE_Print_Structured_Data Type %d", Format->type);
    return_field_level();
    break;
#endif
  }
}


/******************************************************************************
 *
 * FUNCTION: void FAKE_Print_Formatted_Data(stream, format, dataPtr)
 *
 * DESCRIPTION:
 * Print out some data (linearally) according to the given format.
 * "Data_Ptr" is a pointer to some arbitrary data structure.
 * "Format" is the format of the data
 * "Stream" is where to print the data (e.g. stdout)
 *
 * INPUTS: 
 * FILE *stream;
 * FORMAT_PTR format;
 * char **dataPtr;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

static void FAKE_Print_Formatted_Data(FILE *stream, CONST_FORMAT_PTR format, 
			  const void *dataPtr)
{
  Print_Data_Type printer;

  LOCK_CM_MUTEX;
  FAKE_startPrint(stream, &printer);
  FAKE_Print_Structured_Data(stream, &printer, format,
			(CONST_GENERIC_DATA_PTR)dataPtr, 0,
			(FORMAT_PTR)NULL, 0);
  FAKE_endPrint(stream, &printer);
  UNLOCK_CM_MUTEX;
}


/******************************************************************************
 *
 * FUNCTION: void FAKE_PrintArrayFormat(stream, startToken, formatArray, 
 *                                 endToken, keepWithNext)
 *
 * DESCRIPTION:
 *
 * INPUTS: 
 * FILE *stream;
 * char *startToken, *endToken;
 * FORMAT_ARRAY_PTR formatArray;
 * int32 keepWithNext;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

static void FAKE_PrintArrayFormat(FILE *stream, Print_Data_Ptr printer,
			     char *startToken, FORMAT_ARRAY_PTR formatArray,
			     char *endToken, int32 keepWithNext)
{ 
  int32 i, lastElementP;
  
  FAKE_printString(stream, printer, startToken, DEFAULT_KEEP);
  FAKE_Print_Formatter1(stream, printer, formatArray[1].f, keepWithNext);
  FAKE_printString(stream, printer, " : ", DEFAULT_KEEP);
  for (i=2;i<formatArray[0].i;i++) {
    if (printer->truncatedGlobal || truncate_field) {
      FAKE_printString(stream, printer, endToken, DEFAULT_KEEP);
      return;
    } else {
      lastElementP = (i+1 == formatArray[0].i);
      FAKE_printInt(stream, printer, &(formatArray[i].i),
	       ((lastElementP) ? keepWithNext+1 : 1));
      if (lastElementP)
	FAKE_printString(stream, printer, endToken, keepWithNext);
      else
	FAKE_printCommaSpace(stream, printer);
    }
  }
}


/******************************************************************************
 *
 * FUNCTION: void FAKE_Print_Formatter1(stream, format, keepWithNext)
 *
 * DESCRIPTION:
 *
 * INPUTS: 
 * FILE *stream;
 * FORMAT_PTR format;
 * int32 keepWithNext;
 *
 * OUTPUTS: void.
 *
 *****************************************************************************/

static void FAKE_Print_Formatter1(FILE *stream, Print_Data_Ptr printer,
			     CONST_FORMAT_PTR format, int32 keepWithNext)
{ 
  int32 i, last_element_p;
  FORMAT_ARRAY_PTR format_array;
  const char *primFormatName;
  
  switch(format->type) {
  case LengthFMT: 
    FAKE_printInt(stream, printer, &(format->formatter.i), keepWithNext);
    break;
  case PrimitiveFMT: 
    switch(format->formatter.p) {
    case  INT_FMT: primFormatName = INT_FMT_NAME; break;
    case  BOOLEAN_FMT: primFormatName = BOOLEAN_FMT_NAME; break;
    case  FLOAT_FMT: primFormatName = FLOAT_FMT_NAME; break;
    case  DOUBLE_FMT: primFormatName = DOUBLE_FMT_NAME; break;
    case  BYTE_FMT: primFormatName = BYTE_FMT_NAME; break;
    case  TWOBYTE_FMT: primFormatName = TWOBYTE_FMT_NAME; break;
    case  STR_FMT: primFormatName = STR_FMT_NAME; break;
    case  FORMAT_FMT: primFormatName = FORMAT_FMT_NAME; break;
    case  UBYTE_FMT: primFormatName = UBYTE_FMT_NAME; break;
    case  CMAT_FMT: primFormatName = CMAT_FMT_NAME; break;
    case  SMAT_FMT: primFormatName = SMAT_FMT_NAME; break;
    case  IMAT_FMT: primFormatName = IMAT_FMT_NAME; break;
    case  LMAT_FMT: primFormatName = LMAT_FMT_NAME; break;
    case  FMAT_FMT: primFormatName = FMAT_FMT_NAME; break;
    case  DMAT_FMT: primFormatName = DMAT_FMT_NAME; break;
    case  CHAR_FMT: primFormatName = CHAR_FMT_NAME; break;
    case  SHORT_FMT: primFormatName = SHORT_FMT_NAME; break;
    case  LONG_FMT: primFormatName = LONG_FMT_NAME; break;
    case  UCMAT_FMT: primFormatName = UCMAT_FMT_NAME; break;
    case  X_IPC_REF_PTR_FMT: primFormatName = X_IPC_REF_PTR_FMT_NAME; break;
    case  SIUCMAT_FMT: primFormatName = SIUCMAT_FMT_NAME; break;
    case  SICMAT_FMT: primFormatName = SICMAT_FMT_NAME; break;
    case  SISMAT_FMT: primFormatName = SISMAT_FMT_NAME; break;
    case  SIIMAT_FMT: primFormatName = SIIMAT_FMT_NAME; break;
    case  SILMAT_FMT: primFormatName = SILMAT_FMT_NAME; break;
    case  SIFMAT_FMT: primFormatName = SIFMAT_FMT_NAME; break;
    case  SIDMAT_FMT: primFormatName = SIDMAT_FMT_NAME; break;

    case  USHORT_FMT: primFormatName = USHORT_FMT_NAME; break;
    case  UINT_FMT:   primFormatName = UINT_FMT_NAME; break;
    case  ULONG_FMT:  primFormatName = ULONG_FMT_NAME; break;

#ifndef TEST_CASE_COVERAGE
    default: primFormatName = "UNKNOWN"; break;
#endif
    }
    FAKE_printString(stream, printer, primFormatName, keepWithNext);
    break;
    
  case PointerFMT:
    FAKE_printString(stream, printer, "*", DEFAULT_KEEP);
    if (format->formatter.f)
      FAKE_Print_Formatter1(stream, printer, format->formatter.f, keepWithNext);
    else
      FAKE_printString(stream, printer, "!", keepWithNext);
    break;
    
  case StructFMT:
    format_array = format->formatter.a;
    FAKE_printString(stream, printer, "{", DEFAULT_KEEP);
    for (i=1; i<format_array[0].i; i++) {
      if (printer->truncatedGlobal || truncate_field) {
        FAKE_printString(stream, printer, "}", keepWithNext);
        return;
      } else {
	last_element_p = (i+1 == format_array[0].i);
	FAKE_Print_Formatter1(stream, printer, format_array[i].f,
			 ((last_element_p) ? keepWithNext+1 : 1));
	if (last_element_p)
	  FAKE_printString(stream, printer, "}", keepWithNext);
	else
	  FAKE_printCommaSpace(stream, printer);
      }
    }
    break;
    
  case FixedArrayFMT: 
    FAKE_PrintArrayFormat(stream, printer, (char*)"[", format->formatter.a, (char*)"]",
		     keepWithNext);
    break;
    
  case VarArrayFMT: 
    FAKE_PrintArrayFormat(stream, printer, (char*)"<", format->formatter.a, (char*)">",
		     keepWithNext);
    break;
  case NamedFMT:
    FAKE_printString(stream, printer, format->formatter.name, keepWithNext);
    break;
  case BadFormatFMT:
    FAKE_printString(stream, printer, "Bad Format", keepWithNext);
    break;
  case EnumFMT:
    format_array = format->formatter.a;
    FAKE_printString(stream, printer, "{enum ", DEFAULT_KEEP);
    if (format_array[0].i == 2) {
      FAKE_printString(stream, printer, ": ", keepWithNext);
      FAKE_printInt(stream, printer, &(format_array[1].i), keepWithNext);
    } else {
      for (i=2; i<format_array[0].i; i++) {
	if (printer->truncatedGlobal || truncate_field) {
    FAKE_printString(stream, printer, "}", keepWithNext);
	  return;
  } else {
	  last_element_p = (i+1 == format_array[0].i);
	  FAKE_printString(stream, printer, format_array[i].f->formatter.name,
		      ((last_element_p) ? keepWithNext+1 : 1));
	  if (!last_element_p) FAKE_printCommaSpace(stream, printer);
	}
      }
    }
    FAKE_printString(stream, printer, "}", keepWithNext);
    break;
    
#ifndef TEST_CASE_COVERAGE
  default:
    X_IPC_MOD_ERROR1("Unknown FAKE_Print_Formatter1 Type %d", format->type);
    break;
#endif
  }
}

// Level 1
IPC_RETURN_TYPE FAKE_IPC_printData(FORMATTER_PTR formatter, FILE *stream,
			       void *dataptr)
{
    FAKE_Print_Formatted_Data(stream, formatter, dataptr);
    return IPC_OK;
}