
var JXHighlighter=new Class({getOptions:function(){return{};},initialize:function(options){this.setOptions(this.getOptions(),options);if(this.options.initialize)this.options.initialize.call(this);path=this.getBasePath();var els=$$('pre');els.each(function(el){if($chk(el.lang)){new Asset.javascript(path+'jxtended/js/highlight_lang/'+el.lang+'.js',{onload:function(){if(el.lang in highlighters){el.addClass('sourcecode');var inputString;if(el.childNodes.length===0){return;}
else{inputString=el.getText();}
JXHighlighterBuilder.init(document,el);JXHighlighterBuilder.parse(inputString,highlighters[el.lang],JXHighlighterBuilder);JXHighlighterBuilder.close();}
else{}}});}});},getBasePath:function(){var scripts=$$('script');for(i=0;i<scripts.length;i++)
{if(/system\/js\/mootools.js$/.test(scripts[i].src)){return scripts[i].src.replace(/system\/js\/mootools.js$/,'');}}
return'/media/';}});JXHighlighter.implement(new Events);JXHighlighter.implement(new Options);window.addEvent('domready',function(){new JXHighlighter();});var JXHighlighterBuilder={init:function(htmlDocument,element){$(element).empty();this._document=htmlDocument;this._element=element;this._currentText=null;this._documentFragment=htmlDocument.createDocumentFragment();this._currentParent=this._documentFragment;this._span=htmlDocument.createElement('span');this._a=htmlDocument.createElement('a');},startElement:function(style){if(this._currentText!==null){this._currentParent.appendChild(this._document.createTextNode(this._currentText));this._currentText=null;}
var span=this._span.cloneNode(true);span.className=style;this._currentParent.appendChild(span);this._currentParent=span;},endElement:function(){if(this._currentText!==null){if(this._currentParent.className==='url'){var a=this._a.cloneNode(true);a.className='url';var url=this._currentText;if(url.length>0&&url.charAt(0)==='<'&&url.charAt(url.length-1)==='>'){url=url.substr(1,url.length-2);}
if(this.isEmailAddress(url)){url='mailto:'+url;}
a.setAttribute('href',url);a.appendChild(this._document.createTextNode(this._currentText));this._currentParent.appendChild(a);}
else{this._currentParent.appendChild(this._document.createTextNode(this._currentText));}
this._currentText=null;}
this._currentParent=this._currentParent.parentNode;},text:function(s){if(this._currentText===null){this._currentText=s;}
else{this._currentText+=s;}},close:function(){if(this._currentText!==null){this._currentParent.appendChild(this._document.createTextNode(this._currentText));this._currentText=null;}
this._element.appendChild(this._documentFragment);},isEmailAddress:function(url){if(/^mailto:/.test(url)){return false;}
return url.indexOf('@')!==-1;},parse:function(inputString,language,builder){var patternStack={_stack:[],getLength:function(){return this._stack.length;},getTop:function(){var stack=this._stack;var length=stack.length;if(length===0){return undefined;}
return stack[length-1];},push:function(state){this._stack.push(state);},pop:function(){if(this._stack.length===0){throw"pop on empty stack";}
this._stack.pop();}};var pos=0;var currentStyle=undefined;var output=function(s,style){var length=s.length;if(length===0){return;}
if(!style){var pattern=patternStack.getTop();if(pattern!==undefined&&!('state'in pattern)){style=pattern.style;}}
if(currentStyle!==style){if(currentStyle){builder.endElement();}
if(style){builder.startElement(style);}}
builder.text(s);pos+=length;currentStyle=style;};var endOfLinePattern=/\r\n|\r|\n/g;endOfLinePattern.lastIndex=0;var inputStringLength=inputString.length;while(pos<inputStringLength){var start=pos;var end;var startOfNextLine;var endOfLineMatch=endOfLinePattern.exec(inputString);if(endOfLineMatch===null){end=inputStringLength;startOfNextLine=inputStringLength;}
else{end=endOfLineMatch.index;startOfNextLine=endOfLinePattern.lastIndex;}
var line=inputString.substring(start,end);var matchCache=null;var matchCacheState=-1;for(;;){var posWithinLine=pos-start;var pattern=patternStack.getTop();var stateIndex=pattern===undefined?0:pattern.next;var state=language[stateIndex];var numPatterns=state.length;if(stateIndex!==matchCacheState){matchCache=[];}
var bestMatch=null;var bestMatchIndex=-1;for(var i=0;i<numPatterns;i++){var match;if(stateIndex===matchCacheState&&(matchCache[i]===null||posWithinLine<=matchCache[i].index)){match=matchCache[i];}
else{var regex=state[i].regex;regex.lastIndex=posWithinLine;match=regex.exec(line);matchCache[i]=match;}
if(match!==null&&(bestMatch===null||match.index<bestMatch.index)){bestMatch=match;bestMatchIndex=i;}}
matchCacheState=stateIndex;if(bestMatch===null){output(line.substring(posWithinLine),null);break;}
else{if(bestMatch.index>posWithinLine){output(line.substring(posWithinLine,bestMatch.index),null);}
pattern=state[bestMatchIndex];var newStyle=pattern.style;var matchedString;if(newStyle instanceof Array){for(var subexpression=0;subexpression<newStyle.length;subexpression++){matchedString=bestMatch[subexpression+1];output(matchedString,newStyle[subexpression]);}}
else{matchedString=bestMatch[0];output(matchedString,newStyle);}
if('next'in pattern){patternStack.push(pattern);}
else{if('exit'in pattern){patternStack.pop();}
if('exitall'in pattern){while(patternStack.getLength()>0){patternStack.pop();}}}}}
if(currentStyle){builder.endElement();}
currentStyle=undefined;if(endOfLineMatch){builder.text(endOfLineMatch[0]);}
pos=startOfNextLine;}},extend:$extend};