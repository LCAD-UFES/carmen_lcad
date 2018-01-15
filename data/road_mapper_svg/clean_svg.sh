if [ $# -lt 1 ] || [ $1 == "-h" ]
then
        echo "This script removes any absolute reference to the home directory in a SVG file."
        echo "Usage: $0 <SVG_filename1> [...]"
        exit 0
fi

home_slash=$HOME/
count=0

for file in $*
do
	[ -f "$file" ] && [[ $file == *.svg ]] && (( count = count + 1 )) && sed -i s=file\://==g $file ; sed -i s=$home_slash==g $file 
done

if [ $count -gt 0 ]
then
	echo $count SVG files updated.
else
	echo No SVG files updated.
fi

