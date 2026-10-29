#!/bin/bash

# Captura o nome exato deste script (ex: meu-script-astro.sh)
NOME_SCRIPT=$(basename "$0")

echo "Iniciando a substituição de texto dentro dos arquivos (ignorando $NOME_SCRIPT)..."

# 1. Substitui o texto dentro dos arquivos
# O parâmetro '! -name "$NOME_SCRIPT"' impede que ele edite a si mesmo
find . -type f ! -name "$NOME_SCRIPT" -exec sed -i 's/astro/carmen/g; s/Astro/Carmen/g; s/ASTRO/CARMEN/g' {} +

echo "Iniciando a renomeação de arquivos e pastas..."

# 2. Renomeia arquivos e pastas
# A mesma exclusão '! -name "$NOME_SCRIPT"' é adicionada antes da busca pelos nomes
find . -depth ! -name "$NOME_SCRIPT" \( -name '*astro*' -o -name '*Astro*' -o -name '*ASTRO*' \) -print0 | while IFS= read -r -d '' caminho_completo; do
    
    diretorio=$(dirname "$caminho_completo")
    nome_base=$(basename "$caminho_completo")
    
    novo_nome="${nome_base//astro/carmen}"
    novo_nome="${novo_nome//Astro/Carmen}"
    novo_nome="${novo_nome//ASTRO/CARMEN}"
    
    mv "$caminho_completo" "$diretorio/$novo_nome"
    
done

echo "Processo concluído com sucesso!"
