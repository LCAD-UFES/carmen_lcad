#ifndef CARMEN_CPP_GLOBAL_MACROS_H
#define CARMEN_CPP_GLOBAL_MACROS_H

#define PARAM_SET_GET(type, name, qualifier, setqualifier, getqualifier) \
qualifier: type m_##name; \
getqualifier: inline type get##name() const {return m_##name;} \
setqualifier: inline void set##name(type name) {m_##name=name;}  


#define DEFAULT_PARAM_SET_GET(type, name) \
PARAM_SET_GET(type, name, protected, public, public)  


#define PARAM_SET(type, name, qualifier, setqualifier) \
qualifier: type m_##name; \
setqualifier: inline void set##name(type name) {m_##name=name;} 


#define PARAM_GET(type, name, qualifier, getqualifier) \
qualifier: type m_##name; \
getqualifier: inline type get##name() const {return m_##name;} 


#define MEMBER_PARAM_SET_GET(member, type, name, setqualifier, getqualifier) \
getqualifier: inline type get##name() const {return (member).get##name();} \
setqualifier: inline void set##name(type name) { (member).set##name(name);} 


#define MEMBER_PARAM_SET(member, type, name,  setqualifier, getqualifier) \
setqualifier: inline void set##name(type name) { (member).set##name(name);} 


#define MEMBER_PARAM_GET(member, type, name,  setqualifier, getqualifier) \
getqualifier: inline type get##name() const {return (member).get##name();} 


#define STRUCT_PARAM_VIRTUAL_SET_GET(member, type, name, internname, setqualifier, getqualifier) \
getqualifier: inline virtual type get##name() const {return (member).internname;} \
setqualifier: inline virtual void set##name(type name) {(member).internname=name;} 

#define STRUCT_PARAM_SET_GET(member, type, name, internname, setqualifier, getqualifier) \
getqualifier: inline type get##name() const {return (member).internname;} \
setqualifier: inline void set##name(type name) {(member).internname=name;} 

#define DEFAULT_STRUCT_PARAM_SET_GET(member, type, name, internname) \
STRUCT_PARAM_SET_GET(member, type, name, internname,public, public) 


#define STRUCT_PARAM_SET(member, type, name, internname, setqualifier, getqualifier) \
setqualifier: inline void set##name(type name) {(member).name=name;} 


#define STRUCT_PARAM_GET(member, type, name, internname, setqualifier, getqualifier) \
getqualifier: inline type get##name() const {return (member).internname;} 


#define DEFAULT_SIMPLESTRUCT_PARAM_SET_GET(member, type, name) \
STRUCT_PARAM_SET_GET(member, type, name, public, public) 


#define SIMPLESTRUCT_PARAM_SET(member, type, name,  setqualifier, getqualifier) \
setqualifier: inline void set##name(type name) {(member).name=name;} 


#define SIMPLESTRUCT_PARAM_GET(member, type, name,  setqualifier, getqualifier) \
getqualifier: inline type get##name() const {return (member).name;} 



#endif

