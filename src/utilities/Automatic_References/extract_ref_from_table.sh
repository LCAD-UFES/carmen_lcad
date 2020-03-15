cat Referencias.txt | grep "^\[" | cut -d] -f1 | cut -c 2- | egrep '^.{3,11}$' > ref_from_table.txt
cat Texto.txt | grep -oP '\[\K[^\]]+' | sed -e $'s/,/\\\n/g' | sed -e 's/ //' > ref_from_text.txt
sort -o ref_from_text.txt ref_from_text.txt 
sort -o ref_from_table.txt ref_from_table.txt
cp ref_from_text.txt 1.txt
sort -u 1.txt > ref_from_text.txt
rm 1.txt
echo "Duplicados: " > Resultados.txt
uniq -d ref_from_table.txt >> Resultados.txt
echo "" >> Resultados.txt
cp ref_from_table.txt 1.txt
sort -u 1.txt > ref_from_table.txt
rm 1.txt
#comm -23 ref_from_table.txt ref_from_text.txt > Ref_only_in_table.txt
#comm -23 ref_from_text.txt ref_from_table.txt > Ref_only_in_text.txt
echo "Referencias encontradas apenas na tabela: " >> Resultados.txt
comm -23 ref_from_table.txt ref_from_text.txt >> Resultados.txt
echo "" >> Resultados.txt
echo "Referencias encontradas apenas no texto: " >> Resultados.txt
comm -23 ref_from_text.txt ref_from_table.txt >> Resultados.txt

