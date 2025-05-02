# Guia de Contribuição para o drowsinessSensor

Obrigado por considerar contribuir para o drowsinessSensor! Este documento fornece as diretrizes para contribuir com o projeto.

## Como Contribuir

### Reportando Bugs

Se você encontrar um bug, por favor, crie uma issue no GitHub seguindo estas etapas:

1. Verifique se o bug já não foi reportado
2. Utilize o template de bug report e forneça:
   - Descrição clara do problema
   - Passos para reproduzir
   - Comportamento esperado
   - Comportamento observado
   - Informações sobre seu ambiente (versão do Arduino IDE, ESP32-CAM, etc.)
   - Capturas de tela ou logs, se possível

### Sugerindo Melhorias

Para sugerir melhorias:

1. Verifique se sua ideia já não foi sugerida
2. Crie uma issue detalhando:
   - Descrição clara da melhoria
   - Por que essa melhoria seria útil
   - Como implementá-la (se tiver sugestões técnicas)

### Enviando Pull Requests

1. Faça um fork do repositório
2. Crie um branch para sua feature (`git checkout -b feature/nome-da-feature`)
3. Faça suas alterações
4. Verifique se o código compila e testa no hardware, se possível
5. Commit suas alterações (`git commit -m 'Adiciona nova feature'`)
6. Push para o branch (`git push origin feature/nome-da-feature`)
7. Abra um Pull Request

## Diretrizes de Código

### Estilo de Código

- Indente seu código com 2 espaços
- Use nomes de variáveis e funções descritivos
- Adicione comentários para código complexo
- Mantenha funções pequenas e com propósito único
- Siga a convenção de nomenclatura já presente no código

### Documentação

- Documente todas as novas funções e parâmetros
- Atualize a documentação quando modificar funcionalidades existentes
- Adicione comentários explicando o "porquê" quando o código não for autoexplicativo

### Testes

- Teste suas alterações no hardware real (ESP32-CAM)
- Verifique diferentes condições de iluminação, se relevante
- Garanta que suas alterações não ultrapassem a memória disponível no ESP32-CAM

## Recursos do Projeto

- Memória: O ESP32-CAM tem recursos limitados; seja econômico com a RAM
- CPU: Otimize para o melhor desempenho possível
- PSRAM: Utilize quando disponível para buffers grandes

## Otimizações

Ao contribuir, considere:

- Minimizar o uso de memória
- Reduzir o tempo de processamento
- Manter a qualidade da detecção
- Priorizar a estabilidade do sistema

## Processo de Revisão

1. Um mantenedor revisará seu Pull Request
2. Pode haver pedidos de alterações ou esclarecimentos
3. Uma vez aprovado, seu PR será mesclado ao projeto

## Agradecimentos

Agradecemos sua contribuição para tornar o drowsinessSensor melhor! 