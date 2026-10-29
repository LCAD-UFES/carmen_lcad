
# Program with Astro

## Contents

1. [Introduction](#introduction)

2. [ASTRO Style Guide](#astro-style-guide)

    2.1. [The Prime Directive](#the-prime-directive)

    2.2. [Units and Coordinates](#units-and-coordinates)

    2.3. [Coding Style Guide](#coding_style_guide)

3. [Building Your Own Module](#ipc)

## <a name="introduction"></a>1. Introduction

This document is designed to get ASTRO users started in writing new programs for integration into the program set. Because of the diverse skills and habits of programmers, the first section is a Style Guide to ensure compatability and clarity of new programs. The next secions describe the commands for getting information and issuing commands within ASTRO.

## <a name="astro-style-guide"></a>2. ASTRO Style Guide

### <a name="the-prime-directive"></a>2.1. The Prime Directive

You are not the only person who will ever have to read, understand and modify your code.

### <a name="units-and-coordinates"></a>2.2. Units and Coordinates

- Always represent all units in MKS. All distances are always in metres. All angles are always in radians. 

- All floating point numbers should be doubles and all fixed point numbers should be ints.

- All coordinate frames are right-handed. This means that q always increases counter-clockwise, from positive x to positive y. The image bellow shows the coordinates systems our the main hardware:

<div align="center">
  <img src="coordinate_system.png" alt="Descrição da imagem" width="350px">
</div>
    
- Never convert between radians and degrees yourself. Always use:

        astro_radians_to_degrees 

    and

        astro_degrees_to_radians

- Angles are always between -p and p. Never normalize angles yourself. Always use:
    
        astro_normalize_theta

- Never use asin, acos or atan to recover angles distances. Always use atan2.

        theta = atan2(y, x);



### <a name="coding_style_guide"></a>2.3. Coding Style Guide

#### <a name="naming"></a>2.3.1. Naming
The most important naming convention is to expect that all codes of ASTRO are standardized.

1. Local and global variables and functions will be named in  `lower_case_with_underscores`. 
2.  _All_  names should be descriptive, other than loop counter variables like  `i`.

#### <a name="variables"></a>2.3.2. Variables

1. Place a function's variables in the narrowest scope possible, and initialize variables in the declaration, e.g.:

``` c
//Bad -- variable initialized after declaration.
int i; 
i = 5; 
```

``` c
//Good -- variable initialized in declaration.
int i = 5; 
```

2. Declare variables in a scope as local as possible, and as close to the first use as possible, e.g.:
``` c
// Bad -- declaration separate from use.
int jobs = NumJobs();
// More code...
f(jobs);      
```
``` c
// Good -- declaration immediately (or closely) followed by use.
int jobs = NumJobs();
f(jobs);      
```

3. Do not declare or use more variables than are necessary.

4. Numerical constants, e.g. PI number, must not be coded directly. 


#### <a name="comments"></a>2.3.3. Comments

Comments must follow the rules bellow:

1.  Variable, class, and function comments come before the commented component.
2. No specific format is required for comments, but they should be descriptive and clearly explain how to use the function/class being commented.
3.  Prefer  `//`  over  `/*`  and  `*/`.

#### <a name="formatting"></a>2.3.4. Formatting

1.  **Horizontal whitespace:**  Use 1 space around (most) operators. E.g.,  `x = y + z`  as opposed to  `x=y+z`. Exceptions:  `.`,  `->`,  `++`, and  `--`.  Do  _not_  put a space between a function name and its argument or parameter list. E.g., write  `Function(arg1, arg2)`, not  `Function (arg1, arg2)`.
2.  **Braces:**  Always put the opening and ending curly brace on a new line, e.g:

``` c
while (x < 3)
{
  cout << x << ",";
  x++;
}
``` 
3.  **Indentation:**  If you use VSCode just use tab key. If you use other editors, configure it to emit 4 spaces when hit the tab key.
4. No line of code should wrap beyond the right margin. If a line of code becomes too long, break it up into pieces.
5. Good design and clarity take precedence over optimization.

#### <a name="documentation"></a>2.3.5. Documentation

All documentation must be in English, placed immediately before the commented component, and follow these guidelines:

1. __*Consistency*__: Maintain consistency in how variables and functions are documented. This makes the code easier to read and maintain.
2. __*Clarity*__: Always explain the purpose of each variable and function clearly and precisely.
3. __*Update*__: Whenever a function or variable is modified, update its documentation to reflect the changes made.

#### <a name="functions"></a>2.3.5.1. Functions

When creating or updating a function, it is essential to document it so that other developers understand its purpose and how to use it. The comment block should include:

- `@brief`: A brief summary of the function's purpose.
- Description: A detailed explanation of what the function does.
- `@param`: A description of each parameter of the function, including its type and how it is used.
- `@return`: What the function returns and the meaning of the returned value.
- `@note` (Optional): Additional relevant information, such as exceptions, preconditions, side effects, or usage examples.

```c
/**
 * @brief Calculates the average of an integer array.
 * 
 * This function takes a pointer to an integer array and
 * calculates the average of the values contained in the array.
 * 
 * @param array Pointer to the integer array to be processed.
 * @param size Size of the array, i.e., the number of elements.
 * 
 * @return The average of the values in the array. If the array is empty,
 * the function's behavior is undefined.
 * 
 * @note The function assumes that the array has at least one element.
 * Make sure to validate the size before calling this function.
 */
double
calculate_average(int* array, size_t size);
```

#### <a name="global variables"></a>2.3.5.2. Global variables

When a global scope variable is created, it should be accompanied by a descriptive comment block. This comment should include:

- `@brief`: A brief summary of the variable's purpose.
- Description: A detailed explanation of how the variable is used in the code.
- `@note` (Optional): Additional relevant information, such as initialization details, usage restrictions, or any other pertinent notes.

### Exemplo de Documentação para Variável Global

```c
/**
 * @brief Stores the total execution time of the program.
 * 
 * This global variable is used to monitor the total time
 * the program takes from start to finish.
 * 
 * @note It should be initialized at the beginning of the program and 
 * updated periodically throughout the execution.
 */
unsigned long
execution_time;
```

### <a name="ipc"></a>3. Building Your Own Module

The golden rule in the creation of a new module is: DON'T START FROM ZERO. READ ANOTHER MODULES. A example module is located in: `$ASTRO_HOME/src/simple_module_example`.

- There should be three separate files with your module:

        astro_{module_name}_messages.h 
        
    this contains the IPC message definitions.

        astro_{module_name}_interface.h  
        
    this contains the function prototypes for communicating with your module.

        astro_{module_name}_interface.c 
    
    this contains the actual functions for communicating with your module. This file should be compiled into a library.

    We expect that all modules will have well-defined interface libraries that relieve the user of having to worry about marshalling data.

- Every IPC message should have all the necessary data, timestamp and hostname fields. For timestamp and hostname, use respectivelly, `astro_get_time()` and `astro_get_host()`.

#### <a name="getting-data-from-astro"></a>3.1 Creating Your Own Message


#### <a name="getting-data-from-astro"></a>3.2 Modules Communication

ASTRO modules communication uses the publish-subscribe paradigm, where message senders, known as publishers, generate and send information without needing to know the identities of the recipients, known as subscribers. Conversely, subscribers receive messages of interest without needing to know the identities of the publishers. This approach promotes a high degree of decoupling between the parties involved, leading to more efficient and scalable communication.

##### 3.2.1. Subscribing

All of the following messages can be subscribed to by using helper functions in the appropriate xxx_interface library, e.g., base_interface. The helper functions are all of the form:

    void 
    astro_robot_subscribe_xxxx_message(astro_robot_xxx_message *msg,
                                    astro_handler_t handler, 
                                        astro_subscribe_t subscribe_how);

where astro_handler_t and astro_subscribe_t are defined as

    typedef enum {ASTRO_UNSUBSCRIBE, ASTRO_SUBSCRIBE_LATEST, 
                ASTRO_SUBSCRIBE_ALL} astro_subscribe_t;
    typedef void (*astro_handler_t)(void *);

If the msg field of the subscribe function is NULL, then a static message is automatically allocated and returned as the argument to handler, otherwise the message pointer passed to the subscribe function is always used. In all cases, the same memory is re-used for all handled messages of the same message name, and passed as an argument to the handler function.

If the handler field of the subscribe function is NULL, no handler is called, but the memory pointed to by msg is updated regularly. If both handler and msg are both NULL, your program will spend a fair chunk of time doing nothing useful for you.

The subscribe_how field allows the user to either unsubscribe, or to start a new subscription. Subscribing only to the latest message allows the module to fall behind in processing messages without serious consequences. It should be pointed out that subscribing to all messages (ASTRO_SUBSCRIBE_ALL) does not guarantee all messages. Currently, the upper limit for the queue size is 1000 messages. If an IPC process actually subscribes to all messages and falls seriously behind (or wedges), central can run out of memory, or even worse, the TCP stack can overflow. Consequently, the Astro subscription functions limit the maximum message queue size to 1000. A resourceful programmer can increase this queue (or even remove the queue size), but it is not clear this would ever be necessary.

##### 3.2.2. Publishing

The messages you created are published by the functions created in `astro_{module_name}_interface.c`.