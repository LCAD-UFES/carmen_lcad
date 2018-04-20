COMVIEW


To build comview (Ubuntu): 
 1) sudo apt-get install tcl-dev tk-dev texlive bison libxaw7-dev
 2) cd to comview and type make install

To build comview (Fedora): 
 1) yum install tcl-devel tk-devel tetex tetex-latex
 2) cd to comview and type gmake install


If anything goes wrong, cd to doc and gzip *.ps before trying again.

Postscript documentation is doc/comview.pdf

Command line: comview -f <log-filename>

