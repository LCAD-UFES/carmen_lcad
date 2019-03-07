#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <linux/limits.h>
#include <stdarg.h>
#include <ctype.h>
#include <assert.h>
#include "config.h"


#ifndef CONFIG_DIR
#define DEFAULT_CONFIG_PATH "../config/master.cfg"
#else
#define DEFAULT_CONFIG_PATH CONFIG_DIR "/master.cfg"
#endif

#define err(args...) fprintf(stderr, args)

typedef struct _Parser Parser;
typedef struct _ParserFile ParserFile;
typedef struct _ConfigElement ConfigElement;

typedef int (*GetChFunc)(Parser *);

typedef enum {
    ParserTypeFile,
} ParserType;

struct _Parser {
    ParserType  type;
    GetChFunc   get_ch;
    int         extra_ch;
};

struct _ParserFile {
    Parser  p;
    char *  filename;
    FILE *  file;
    int     row;
    int     col;
    int     in_comment;
};

typedef enum {
    TokInvalid,
    TokIdentifier,
    TokOpenStruct,
    TokCloseStruct,
    TokOpenArray,
    TokCloseArray,
    TokArraySep,
    TokAssign,
    TokString,
    TokEndStatement,
    TokCast,
    TokEOF,
} ConfigToken;

typedef enum {
    ConfigContainer,
    ConfigArray
} ConfigType;

typedef enum {
    ConfigDataString,
    ConfigDataInt,
    ConfigDataBool,
    ConfigDataDouble
} ConfigDataType;

struct _ConfigElement {
    ConfigType type;
    ConfigDataType data_type;
    ConfigElement * parent;
    char * name;
    ConfigElement * next;
    ConfigElement * children;
    int num_values;
    char ** values;
};

struct _Config {
    ConfigElement * root;
};

/* Prints an error message, preceeded by useful context information from the
 * parser (i.e. line number). */
static int
print_msg (Parser * p, char * format, ...)
{
    va_list args;
    if (p->type == ParserTypeFile) {
        ParserFile * pf = (ParserFile *) p;
        char * fname = strrchr (pf->filename, '/');
        if (fname)
            fname++;
        else
            fname = pf->filename;
        fprintf (stderr, "%s:%d ", fname, pf->row+1);
        va_start (args, format);
        vfprintf (stderr, format, args);
        va_end (args);
        return 0;
    }
    return -1;
}

/* Get the next character from a file, while converting all forms of
 * whitespace into plain spaces and stripping comments.
 *
 * Returns the next printable character on success, 0 on EOF, -1 on
 * error.
 */
static int
get_ch_file (Parser * p)
{
    ParserFile * pf = (ParserFile *) p;
    int ch;

    /* If a character has been put back with unget_ch, get it. */
    if (p->extra_ch) {
        ch = p->extra_ch;
        p->extra_ch = 0;
        return ch;
    }

    while ((ch = getc (pf->file)) != EOF) {
        if (ch == '\n') {
            pf->row++;
            pf->col = 0;
            pf->in_comment = 0;
            return ' ';
        }
        if (ch == '#')
            pf->in_comment = 1;
        if (pf->in_comment)
            continue;

        pf->col++;
        if (isspace (ch))
            return ' ';
        if (!isprint (ch)) {
            print_msg (p, "Error: Non-printable character 0x%02x\n", ch);
            return -1;
        }
        return ch;
    }
    return 0;
}

/* Returns a previously gotten character to the buffer, so it will be gotten
 * next.  This function cannot be used more than once before getting the
 * next character. */
static int
unget_ch (Parser * p, int ch)
{
    p->extra_ch = ch;
    return 0;
}

/* Get the next token from the parser.  All information about what constitutes
 * a token is expressed in this function.
 *
 * The type of token is stored in tok.  The actual text of the token is stored
 * in str, a buffer of length len passed by the caller.
 */
static int
get_token (Parser * p, ConfigToken * tok, char * str, int len)
{
    int ch, end_ch=0, c=0, escape=0;
    *tok = TokInvalid;

    /* Skip whitespace (all whitespace converted to ' ' already) */
    while ((ch = p->get_ch (p)) == ' ');

    if (ch == -1)
        return -1;

    if (len < 4) {
        fprintf (stderr, "Error: not enough space to store token\n");
        return -1;
    }

    if (ch == 0) {
        snprintf (str, len, "EOF");
        *tok = TokEOF;
        return 0;
    }

    str[c++] = ch;
    if (ch == ';') {
        *tok = TokEndStatement;
        goto finish;
    }
    if (ch == '=') {
        *tok = TokAssign;
        goto finish;
    }
    if (ch == '[') {
        *tok = TokOpenArray;
        goto finish;
    }
    if (ch == ']') {
        *tok = TokCloseArray;
        goto finish;
    }
    if (ch == '{') {
        *tok = TokOpenStruct;
        goto finish;
    }
    if (ch == '}') {
        *tok = TokCloseStruct;
        goto finish;
    }
    if (ch == ',') {
        *tok = TokArraySep;
        goto finish;
    }

    /* A string always starts with a double quote */
    if (ch == '\"') {
        c--;
        *tok = TokString;
        end_ch = '\"';
        escape = '\\';
    }
    /* A cast always starts with an open paren.
     * TODO: this will need to be tokenized further once the cast is actually
     * used for something. */
    if (ch == '(') {
        c--;
        *tok = TokCast;
        end_ch = ')';
        escape = 0;
    }
    /* An identifier starts with alpha-numeric text or a few symbols */
    if (isalnum (ch) || ch == '_' || ch == '-' || ch == '.') {
        *tok = TokIdentifier;
        end_ch = 0;
        escape = 0;
    }

    if (*tok == TokInvalid) {
        print_msg (p, "Error: Unexpected character \"%c\"\n", ch);
        return -1;
    }

    /* Read the remaining text of a string, cast, or identifier */
    int prev_ch = 0;
    while (1) {
        ch = p->get_ch (p);
        /* An identifier is terminated as soon as we see a character which
         * itself cannot be part of an identifier. */
        if (*tok == TokIdentifier &&
                !isalnum (ch) && ch != '_' && ch != '-' && ch != '.') {
            if (ch != 0)
                unget_ch (p, ch);
            goto finish;
        }
        if (ch == 0) {
            print_msg (p, "Error: Expected '%c' but got end-of-file\n",
                    end_ch);
            return -1;
        }
        /* Strings or casts are terminated when their respective end
         * characters are read, as long as the character is not escaped. */
        if (ch == end_ch && prev_ch != escape)
            goto finish;
        prev_ch = ch;
        str[c++] = ch;

        if (c >= len) {
            print_msg (p, "Error: Token is too large for buffer (%d bytes)\n",
                    len);
            return -1;
        }
    }
finish:
    str[c] = '\0';
    return 0;
}

static ConfigElement *
new_element (const char * name)
{
    ConfigElement * el;

    el = malloc (sizeof (ConfigElement));
    memset (el, 0, sizeof (ConfigElement));
    if (name)
        el->name = strdup (name);
    el->data_type = ConfigDataString;

    return el;
}

static void
free_element (ConfigElement * el)
{
    free (el->name);
    ConfigElement * child, * next;
    for (child = el->children; child; child = next) {
        next = child->next;
        free_element (child);
    }
    int i;
    for (i = 0; i < el->num_values; i++)
        free (el->values[i]);
    free (el->values);
    free (el);
}

#if 0
/* Debugging function that prints all tokens sequentially from a file */
static int
print_all_tokens (Parser * p)
{
    ConfigToken tok;
    char str[256];

    while (get_token (p, &tok, str, sizeof (str)) == 0) {
        printf ("tok %d: %s\n", tok, str);
        if (tok == TokEOF)
            return 0;
    }
    return -1;
}
#endif

/* Appends child to the list of el's children. */
static int
add_child (Parser * p, ConfigElement * el, ConfigElement * child)
{
    ConfigElement ** nptr;
    for (nptr = &el->children; *nptr != NULL; nptr = &((*nptr)->next));
    *nptr = child;
    child->next = NULL;
    child->parent = el;
    return 0;
}

/* Appends str to the list of el's values. */
static int
add_value (Parser * p, ConfigElement * el, const char * str)
{
    int n = el->num_values;
    el->values = realloc (el->values, (n + 1) * sizeof (char *));
    el->values[n] = strdup (str);
    el->num_values = n + 1;
    return 0;
}

/* Parses the interior portion of an array (the part after the leading "["),
 * adding any values to the array's list of values.  Terminates when the
 * trailing "]" is found.
 */
static int
parse_array (Parser * p, ConfigElement * el)
{
    ConfigToken tok;
    char str[256];

    while (1) {
        if (get_token (p, &tok, str, sizeof (str)) < 0)
            goto fail;

        if (tok == TokIdentifier || tok == TokString) {
            add_value (p, el, str);
        }
        else if (tok == TokCloseArray) {
            return 0;
        }
        else {
            print_msg (p, "Error: unexpected token \"%s\", expected value or "
                    "end of array\n", str);
            goto fail;
        }

        if (get_token (p, &tok, str, sizeof (str)) < 0)
            goto fail;

        if (tok == TokArraySep) {
            /* do nothing */
        }
        else if (tok == TokCloseArray) {
            return 0;
        }
        else {
            print_msg (p, "Error: unexpected token \"%s\", expected comma or "
                    "end of array\n", str);
            goto fail;
        }
    }

fail:
    return -1;
}

/* Parses the right-hand side of an assignment (after the equal sign).
 * Checks for any preceeding optional cast, and then parses the value of the
 * assignment.  Terminates when the trailing semicolon is found.
 */
static int
parse_right_side (Parser * p, ConfigElement * el)
{
    ConfigToken tok;
    char str[256];

    if (get_token (p, &tok, str, sizeof (str)) != 0)
        goto fail;

    /* Allow an optional cast preceeding the right-hand side */
    if (tok == TokCast) {
        /* Cast is currently ignored */
        if (get_token (p, &tok, str, sizeof (str)) != 0)
            goto fail;
    }

    if (tok == TokIdentifier || tok == TokString) {
        add_value (p, el, str);
    }
    else if (tok == TokOpenArray) {
        if (parse_array (p, el) < 0)
            goto fail;
    }
    else {
        print_msg (p, "Error: unexpected token \"%s\", expected right-hand "
                "side\n", str);
        goto fail;
    }

    if (get_token (p, &tok, str, sizeof (str)) != 0)
        goto fail;

    if (tok != TokEndStatement) {
        print_msg (p, "Error: unexpected token \"%s\", expected semicolon\n", str);
        goto fail;
    }
    
    return 0;

fail:
    return -1;
}

/* Parses the interior a container (the portion after the "{").  Any
 * assignment statements or enclosed containers are recursively parsed.
 * Terminates when end_token is found, which should be TokEOF for the
 * top-level container and TokCloseStruct for everything else. */
static int
parse_container (Parser * p, ConfigElement * cont, ConfigToken end_token)
{
    ConfigToken tok;
    char str[256];
    ConfigElement * child = NULL;

    while (get_token (p, &tok, str, sizeof (str)) == 0) {
        //printf ("t %d: %s\n", tok, str);
        if (!child && tok == TokIdentifier) {
            child = new_element (str);
        }
        else if (child && tok == TokAssign) {
            child->type = ConfigArray;
            if (parse_right_side (p, child) < 0)
                goto fail;
            add_child (p, cont, child);
            child = NULL;
        }
        else if (child && tok == TokOpenStruct) {
            child->type = ConfigContainer;
            if (parse_container (p, child, TokCloseStruct) < 0)
                goto fail;
            add_child (p, cont, child);
            child = NULL;
        }
        else if (!child && tok == end_token)
            return 0;
        else {
            print_msg (p, "Error: unexpected token \"%s\"\n", str);
            goto fail;
        }
    }

fail:
    if (child) {
        free_element (child);
    }
    return -1;
}

static int
print_array (ConfigElement * el, int indent)
{
    printf ("%*s%s = [", indent, "", el->name);

    int i;
    for (i = 0; i < el->num_values; i++) {
        printf ("\"%s\", ", el->values[i]);
    }
    printf ("];\n");
    return 0;
}

static int
print_container (ConfigElement * el, int indent)
{
    ConfigElement * child;

    printf ("%*s%s {\n", indent, "", el->name);

    for (child = el->children; child; child = child->next) {
        if (child->type == ConfigContainer)
            print_container (child, indent + 4);
        else if (child->type == ConfigArray)
            print_array (child, indent + 4);
        else {
            fprintf (stderr, "Error: unknown child (%d)\n", child->type);
            return -1;
        }
    }

    printf ("%*s}\n", indent, "");
    return 0;
}

/* Prints the contents of a configuration file's parse tree to standard
 * output. */
int
config_print (Config * conf)
{
    ConfigElement * child, * root;

    root = conf->root;

    for (child = root->children; child; child = child->next) {
        if (child->type == ConfigContainer)
            print_container (child, 0);
        else if (child->type == ConfigArray)
            print_array (child, 0);
        else {
            fprintf (stderr, "Error: unknown child (%d)\n", child->type);
            return -1;
        }
    }
    return 0;
}

Config *
config_parse_file (FILE * f, char * filename)
{
    ParserFile pf;

    memset (&pf, 0, sizeof (ParserFile));
    pf.p.get_ch = get_ch_file;
    pf.p.type = ParserTypeFile;
    pf.file = f;
    pf.filename = filename;

    ConfigElement * root;

    root = new_element (NULL);
    root->type = ConfigContainer;
    if (parse_container (&pf.p, root, TokEOF) < 0) {
        free_element (root);
        return NULL;
    }

    Config * conf;
    conf = malloc (sizeof (Config));
    conf->root = root;

    return conf;
}

Config *
config_parse_default (void)
{
    char path[PATH_MAX];
    int status = config_get_default_src (path, sizeof (path));
    assert (0 == status);

    printf ("Using %s for configuration.\n", path);
    FILE * f = fopen (path, "r");
    if (!f) {
        fprintf (stderr, "Error: failed to open %s\n", path);
        fprintf (stderr, "ERROR (%s:%d):  Unable to load config file.\n"
                "    If this is a clean svn checkout, then you must create a\n"
                "    symbolic link to master.cfg from the actual config file used.\n"
                "    e.g.\n"
                "\n"
                "   $ cd dgc/software/trunk/config\n"
                "   $ ln -s lr3.cfg master.cfg\n"
                "\n", __FILE__, __LINE__);
        return NULL;
    }

    Config * conf = config_parse_file (f, path);
    fclose (f);
    return conf;
}

int 
config_get_default_src (char *buf, int buflen)
{
    char * path = getenv ("DGC_CONFIG_PATH");
    if (!path) path = DEFAULT_CONFIG_PATH;
    if (strlen (path) > (unsigned int) buflen - 1) return -1;
    strcpy (buf, path);
    return 0;
}

void
config_free (Config * conf)
{
    free_element (conf->root);
    free (conf);
}

static ConfigElement *
find_key (ConfigElement * el, const char * key, int inherit)
{
    size_t len = strcspn (key, ".");
    char str[len+1];
    memcpy (str, key, len);
    str[len] = '\0';

    const char * remainder = NULL;
    if (key[len] == '.')
        remainder = key + len + 1;

    ConfigElement * child;
    for (child = el->children; child; child = child->next) {
        if (!strcmp (str, child->name)) {
            if (remainder)
                return find_key (child, remainder, inherit);
            else
                return child;
        }
    }
    if (inherit && !remainder && el->parent)
        return find_key (el->parent, str, inherit);
    else
        return NULL;
}

static int
cast_to_int (const char * key, const char * val, int * out)
{
    char * end;
    *out = strtol (val, &end, 0);
    if (end == val || *end != '\0') {
        fprintf (stderr, "Error: key \"%s\" (\"%s\") did not cast "
                "properly to int\n", key, val);
        return -1;
    }
    return 0;
}

static int
cast_to_boolean (const char * key, const char * val, int * out)
{
    if (!strcasecmp (val, "y") || !strcasecmp (val, "yes") ||
            !strcasecmp (val, "true") || !strcmp (val, "1"))
        *out = 1;
    else if (!strcasecmp (val, "n") || !strcasecmp (val, "no") ||
            !strcasecmp (val, "false") || !strcmp (val, "0"))
        *out = 0;
    else {
        fprintf (stderr, "Error: key \"%s\" (\"%s\") did not cast "
                "properly to boolean\n", key, val);
        return -1;
    }
    return 0;
}

static double
cast_to_double (const char * key, const char * val, double * out)
{
    char * end;
    *out = strtod (val, &end);
    if (end == val || *end != '\0') {
        fprintf (stderr, "Error: key \"%s\" (\"%s\") did not cast "
                "properly to double\n", key, val);
        return -1;
    }
    return 0;
}

#define PRINT_KEY_NOT_FOUND(key) \
    err("WARNING: Config: could not find key %s!\n", (key));


int 
config_has_key (Config *conf, const char *key)
{
    return (find_key (conf->root, key, 1) != NULL);
}

int
config_get_num_subkeys (Config * conf, const char * containerKey)
{
  ConfigElement* el = conf->root;
  if ((NULL != containerKey) && (0 < strlen(containerKey)))
    el = find_key (conf->root, containerKey, 1);
  if (NULL == el)
    return -1;

  int count = 0;
  ConfigElement* child;
  for (child = el->children; child; child = child->next)
    ++count;

  return count;
}


char **
config_get_subkeys (Config * conf, const char * containerKey)
{
    ConfigElement* el = conf->root;
    if ((NULL != containerKey) && (0 < strlen(containerKey)))
        el = find_key (conf->root, containerKey, 1);
    if (NULL == el)
        return NULL;

    int count = 0;
    ConfigElement* child;
    for (child = el->children; child; child = child->next, ++count);

    char **result = calloc (count + 1, sizeof (char*));

    int i = 0;
    for (child = el->children; child; child = child->next) {
        result[i] = strdup (child->name);
        i++;
    }

    return result;
}



int
config_get_int (Config * conf, const char * key, int * val)
{
    ConfigElement * el = find_key (conf->root, key, 1);
    if (!el || el->type != ConfigArray || el->num_values < 1) {
        return -1;
    }
    return cast_to_int (key, el->values[0], val);
}

int
config_get_boolean (Config * conf, const char * key, int * val)
{
    ConfigElement * el = find_key (conf->root, key, 1);
    if (!el || el->type != ConfigArray || el->num_values < 1) {
        return -1;
    }
    return cast_to_boolean (key, el->values[0], val);
}

int
config_get_double (Config * conf, const char * key, double * val)
{
    ConfigElement * el = find_key (conf->root, key, 1);
    if (!el || el->type != ConfigArray || el->num_values < 1) {
        return -1;
    }
    return cast_to_double (key, el->values[0], val);
}

double config_get_double_or_fail (Config *conf, const char *key)
{
    double v;
    int res = config_get_double(conf, key, &v);
    if (res) {
        printf("Missing config key: %s\n", key);
        abort();
    }

    return v;
}

int
config_get_str (Config * conf, const char * key, char ** val)
{
    ConfigElement * el = find_key (conf->root, key, 1);
    if (!el || el->type != ConfigArray || el->num_values < 1) {
        return -1;
    }
    *val = el->values[0];
    return 0;
}

char *config_get_str_or_fail (Config *conf, const char *key)
{
    char * str;
    if (config_get_str (conf, key, &str) == 0)
        return str;
    else {
        printf("Missing config key: %s\n", key);
        abort();
    }
}

int
config_get_int_or_default (Config * conf, const char * key, int def)
{
    int val;
    if (config_get_int (conf, key, &val) == 0)
        return val;
    else
        return def;
}

int
config_get_boolean_or_default (Config * conf, const char * key, int def)
{
    int val;
    if (config_get_boolean (conf, key, &val) == 0)
        return val;
    else
        return def;
}

double
config_get_double_or_default (Config * conf, const char * key, double def)
{
    double val;
    if (config_get_double (conf, key, &val) == 0)
        return val;
    else
        return def;
}

char *
config_get_str_or_default (Config * conf, const char * key, char * def)
{
    char * str;
    if (config_get_str (conf, key, &str) == 0)
        return str;
    else
        return def;
}

int
config_get_int_array (Config * conf, const char * key, int * vals, int len)
{
    ConfigElement * el = find_key (conf->root, key, 1);
    if (!el || el->type != ConfigArray) {
        return -1;
    }
    int i;
    for (i = 0; i < el->num_values; i++) {
        if (i == len)
            break;
        if (cast_to_int (key, el->values[i], vals + i) < 0) {
            err("WARNING: Config: cast error parsing int array %s\n", key);
            return -1;
        }
    }
    if( i < len ) {
        err("WARNING: Config: only read %d of %d values for integer array\n"
            "         %s\n", i, len, key);
    }
    return i;
}

int
config_get_boolean_array (Config * conf, const char * key, int * vals, int len)
{
    ConfigElement * el = find_key (conf->root, key, 1);
    if (!el || el->type != ConfigArray) {
        return -1;
    }
    int i;
    for (i = 0; i < el->num_values; i++) {
        if (i == len)
            break;
        if (cast_to_boolean (key, el->values[i], vals + i) < 0) {
            err("WARNING: Config: cast error parsing boolean array %s\n", key);
            return -1;
        }
    }
    if( i < len ) {
        err("WARNING: Config: only read %d of %d values for boolean array\n"
            "         %s\n", i, len, key);
    }
    return i;
}

int
config_get_double_array (Config * conf, const char * key, double * vals, int len)
{
    ConfigElement * el = find_key (conf->root, key, 1);
    if (!el || el->type != ConfigArray) {
        return -1;
    }
    int i;
    for (i = 0; i < el->num_values; i++) {
        if (i == len)
            break;
        if (cast_to_double (key, el->values[i], vals + i) < 0) {
            err("WARNING: Config: cast error parsing double array %s\n", key);
            return -1;
        }
    }
    if( i < len ) {
        err("WARNING: Config: only read %d of %d values for double array\n"
            "         %s\n", i, len, key);
    }
    return i;
}

int
config_get_str_array (Config * conf, const char * key, char ** vals, int len)
{
    ConfigElement * el = find_key (conf->root, key, 1);
    if (!el || el->type != ConfigArray) {
        return -1;
    }
    int i;
    for (i = 0; i < el->num_values; i++) {
        if (i == len)
            break;
        vals[i] = el->values[i];
    }
    if( i < len ) {
        err("WARNING: Config: only read %d of %d values for string array\n"
            "         %s\n", i, len, key);
    }
    return i;
}

int 
config_get_array_len (Config *conf, const char * key)
{
    ConfigElement * el = find_key (conf->root, key, 1);
    if (!el || el->type != ConfigArray) {
        return -1;
    }
    return el->num_values;
}

char **
config_get_str_array_alloc (Config * conf, const char * key)
{
    ConfigElement * el = find_key (conf->root, key, 1);
    if (!el || el->type != ConfigArray) {
        return NULL;
    }

    // + 1 so that the list is null terminated.
    char **data = calloc(el->num_values + 1, sizeof(char*));
    
    int i;
    for (i = 0; i < el->num_values; i++) {
        data[i] = strdup(el->values[i]);
    }

    return data;
}

void config_str_array_free(char **data)
{
    int idx = 0;
    while (data[idx] != NULL)
        free(data[idx++]);

    free(data);
}

Config *
config_alloc( void )
{
  ConfigElement * root;
  root = new_element (NULL);
  root->type = ConfigContainer;

  Config * conf;
  conf = malloc (sizeof (Config));
  conf->root = root;

  return conf;
}

static ConfigElement *
create_key(ConfigElement * el,
           const char * key)
{
  size_t len = strcspn (key, ".");
  char str[len+1];
  memcpy (str, key, len);
  str[len] = '\0';

  const char * remainder = NULL;
  if (key[len] == '.')
    remainder = key + len + 1;

  ConfigElement * child;
  for (child = el->children; child; child = child->next) {
    if (!strcmp (str, child->name)) {
      if (remainder)
        return create_key (child, remainder);
      else
        return child;
    }
  }

  child = new_element (str);
  add_child (NULL, el, child);
  if (remainder) {
    child->type = ConfigContainer;
    return create_key (child, remainder);
  }
  else {
    child->type = ConfigArray;
    return child;
  }
}


/*
 * Functions for setting key/value pairs
 */

static int
set_value (Config * conf,
           const char * key,
           const char * val)
{
  ConfigElement * el = find_key (conf->root, key, 0);
  if (el == NULL)
    el = create_key (conf->root, key);
  else if (el->type != ConfigArray)
    return -1;

  if (el->num_values < 1)
    add_value (NULL, el, val);
  else {
    free (el->values[0]);
    el->values[0] = strdup (val);
  }
  return 1;
}


int
config_set_int (Config * conf,
                const char * key,
                int val)
{
  char str[16];
  sprintf (str, "%d", val);
  return set_value (conf, key, str);
}

int
config_set_boolean (Config * conf,
                    const char * key,
                    int val)
{
  return set_value (conf, key, (val == 0 ? "false" : "true"));
}

int
config_set_double (Config * conf,
                   const char * key,
                   double val)
{
  char str[32];
  sprintf (str, "%f", val);
  return set_value (conf, key, str);
}

int
config_set_str (Config * conf,
                const char * key,
                char * val)
{
  return set_value (conf, key, val);
}


/*
 * Functions for setting array of values
 */

int
config_set_int_array (Config * conf,
                      const char * key,
                      int * vals,
                      int len)
{
  char* str;
  char single_val[16];
  int string_len = 1;
  int single_len;
  int i;

  str = malloc(1);
  str[0] = '\0';
  for (i = 0; i < len; ++i) {
    if (i < len-1)
      sprintf (single_val, "%d,", vals[i]);
    else
      sprintf (single_val, "%d", vals[i]);
    single_len = strlen(single_val);
    str = realloc (str, string_len + single_len);
    strcat(str,single_val);
    string_len += single_len;
  }

  int ret_val = set_value (conf, key, str);
  free (str);
  return ret_val;
}

int
config_set_boolean_array (Config * conf,
                          const char * key,
                          int * vals,
                          int len)
{
  char* str;
  char single_val[16];
  int string_len = 1;
  int single_len;
  int i;
  char val_str[8];

  str = malloc(1);
  str[0] = '\0';
  for (i = 0; i < len; ++i) {
    strcpy(val_str, (vals[i] == 0 ? "false" : "true"));
    if (i < len-1)
      sprintf (single_val, "%s,", val_str);
    else
      sprintf (single_val, "%s", val_str);
    single_len = strlen(single_val);
    str = realloc (str, string_len + single_len);
    strcat(str,single_val);
    string_len += single_len;
  }

  int ret_val = set_value (conf, key, str);
  free (str);
  return ret_val;
}

int
config_set_double_array (Config * conf,
                         const char * key,
                         double * vals,
                         int len)
{
  char* str;
  char single_val[32];
  int string_len = 1;
  int single_len;
  int i;

  str = malloc(1);
  str[0] = '\0';
  for (i = 0; i < len; ++i) {
    if (i < len-1)
      sprintf (single_val, "%f,", vals[i]);
    else
      sprintf (single_val, "%f", vals[i]);
    single_len = strlen(single_val);
    str = realloc (str, string_len + single_len);
    strcat(str,single_val);
    string_len += single_len;
  }

  int ret_val = set_value (conf, key, str);
  free (str);
  return ret_val;
}

int
config_set_str_array (Config * conf,
                      const char * key,
                      char ** vals,
                      int len)
{
  char* str;
  char single_val[256];
  int string_len = 1;
  int single_len;
  int i;

  str = malloc(1);
  str[0] = '\0';
  for (i = 0; i < len; ++i) {
    if (i < len-1)
      sprintf (single_val, "%s,", vals[i]);
    else
      sprintf (single_val, "%s", vals[i]);
    single_len = strlen(single_val);
    str = realloc (str, string_len + single_len);
    strcat(str,single_val);
    string_len += single_len;
  }

  int ret_val = set_value (conf, key, str);
  free (str);
  return ret_val;
}
