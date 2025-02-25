import sys

def main():
    minha_str = input("Digite uma string: ")
    input_str = minha_str.strip()
    # Remove os caracteres '{' e '}' e espaços em branco
    types_str = input_str.strip('{}').replace(' ', '')
    # Divide os tipos separados por vírgula
    types = types_str.split(',') if types_str else []
    
    # Dicionário com informações de tamanho e alinhamento para cada tipo
    type_info = {
        "string": {"size": 8, "align": 8},
        "int": {"size": 4, "align": 4},
        "double": {"size": 8, "align": 8},
        # Adicione outros tipos conforme necessário
    }
    
    current_offset = 0
    max_alignment = 0
    
    for t in types:
        if t not in type_info:
            print(f"Erro: Tipo desconhecido '{t}'")
            return
        info = type_info[t]
        align = info["align"]
        size = info["size"]
        # Calcula o padding necessário para alinhar o membro atual
        padding = (-current_offset) % align
        current_offset += padding
        current_offset += size
        # Atualiza o maior alinhamento encontrado
        if align > max_alignment:
            max_alignment = align
    
    # Adiciona o padding final para garantir que o tamanho total seja múltiplo do maior alinhamento
    if max_alignment > 0:
        padding_end = (-current_offset) % max_alignment
        current_offset += padding_end
    
    print(current_offset)

if __name__ == "__main__":
    main()