while read LINE; do
	FILE=${LINE##*/}
	TYPE=${FILE##*.}
	if [[ "$TYPE" != "$FILE" ]]; then
		TYPE=".$TYPE"
		if [[ $ALL_TYPES != *$TYPE* ]]; then
			ALL_TYPES="$ALL_TYPES $TYPE"
			echo "$TYPE"
		fi
	fi
done < caco.txt

echo "$ALL_TYPES"

