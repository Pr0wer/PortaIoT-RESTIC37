#include <stdio.h>             
#include <string.h>             
#include <stdlib.h>

#include "pico/stdlib.h"         
#include "hardware/pwm.h" 
#include "hardware/timer.h"     
#include "pico/cyw43_arch.h"     

#include "lwip/pbuf.h"           
#include "lwip/tcp.h"           
#include "lwip/netif.h"   

#include "lib/ssd1306.h"
#include "lib/ws2812b.h"

// Credenciais WIFI - Tome cuidado se publicar no github!
#define WIFI_SSID "MEU_SSID"
#define WIFI_PASSWORD "MINHA_SENHA"

#define ABERTA "Aberta"
#define FECHADA "Fechada"

#define MAX_TENTATIVAS 3

// Tamanho da senha em caracteres
#define PASSWORD_SIZE 4
#define PASSWORD_ALARM_SIZE 8
#define BUFFER1_SIZE PASSWORD_SIZE + 1
#define BUFFER2_SIZE PASSWORD_ALARM_SIZE + 1

// Pinos
const uint led_green_pin = 11;
const uint led_red_pin = 13;
const uint buzzer_pin = 21;

const uint16_t wrap_t= 1000;
const float clkdiv_t = 125.0;
static bool buzzer_on = false;
uint slice;

static bool alarme_on = false;

static char senha[BUFFER1_SIZE];
static char senha_alarme[BUFFER2_SIZE];
static bool setup = true;

static bool atualizar_display = true; // Flag para atualizar o displaySSD1306 (Já começa ativado para inicialização)
static bool atualizar_matriz = true; // Flag para atualizar a Matriz de LEDs (Já começa ativado para inicialização)

static char *estado_porta = FECHADA; // Indica se o usuário terminou de digitar uma senha ou não
static bool led_callback_on = false;
static bool buzzer_callback_on = false;
static char *msg_erro = "";
static char *msg_alarme = "";

static char str_status[20];

static uint8_t tentativas = 0;

static struct repeating_timer timer;

Rgb frameAlarme[MATRIZ_ROWS][MATRIZ_COLS] = 
{
    {{1, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 0, 0}, {1, 0, 0}},
    {{1, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 0, 0}, {1, 0, 0}},
    {{1, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 0, 0}, {1, 0, 0}},
    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
    {{1, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 0, 0}, {1, 0, 0}}
};
Rgb frameAberto[MATRIZ_ROWS][MATRIZ_COLS] = 
{
    {{0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 0, 0}},
    {{0, 1, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
    {{0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}},
    {{0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}},
    {{0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}}
};
Rgb frameFechado[MATRIZ_ROWS][MATRIZ_COLS] = 
{
    {{0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}},
    {{0, 1, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 1, 0}},
    {{0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}},
    {{0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}},
    {{0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}}
};


// Trecho para modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
  reset_usb_boot(0, 0);
}

// Headers de função
void inicializarLed(void); // Inicializa os Pinos GPIO para acionamento dos LEDs da BitDogLab
void inicializarPwm(uint pino); // Inicializa o PWM no pino especificado
void limparBuffer(int size, char array[size]); // Limpa o que foi armazenado em um array de caracteres
uint8_t escreverBuffer(int size, char array[size], char *values, char break_point); // Escreve em um buffer até encontrar um caractere específico
int64_t led_result_callback(alarm_id_t id, void* user_data); // Callback para transitar do modo de validação ao de digitação
bool buzzer_callback(struct repeating_timer *t); // Callback para funcionamento do buzzer durante o alarme
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err); // Função de callback ao aceitar conexões TCP
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err); // Função de callback para processar requisições HTTP
void user_request(char *request); // Tratamento do request do usuário

// Função principal
int main()
{
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    // Aqui termina o trecho para modo BOOTSEL com botão B

    stdio_init_all();

    // Inicializa o display
    ssd1306_t ssd;
    ssd1306_i2c_init(&ssd);
    ssd1306_draw_string(&ssd, "Inicializando...", 8, 8);
    ssd1306_send_data(&ssd);

    // Inicializa a matriz
    inicializarMatriz();

    // Inicializar os Pinos GPIO para acionamento dos LEDs da BitDogLab
    inicializarLed();

    inicializarPwm(buzzer_pin);
    pwm_set_enabled(slice, true);

    //Inicializa a arquitetura do cyw43
    while (cyw43_arch_init())
    {
        printf("Falha ao inicializar Wi-Fi\n");
        sleep_ms(100);
        return -1;
    }

    // Ativa o Wi-Fi no modo Station, de modo a que possam ser feitas ligações a outros pontos de acesso Wi-Fi.
    cyw43_arch_enable_sta_mode();

    // Conectar à rede WiFI - fazer um loop até que esteja conectado
    printf("Conectando ao Wi-Fi...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000))
    {
        printf("Tentando conectar ao Wi-Fi...\n");
        sleep_ms(100);
    }
    printf("Conectado ao Wi-Fi\n");

    // Caso seja a interface de rede padrão - imprimir o IP do dispositivo.
    if (netif_default)
    {
        printf("IP do dispositivo: %s\n", ipaddr_ntoa(&netif_default->ip_addr));
    }

    // Configura o servidor TCP - cria novos PCBs TCP. É o primeiro passo para estabelecer uma conexão TCP.
    struct tcp_pcb *server = tcp_new();
    if (!server)
    {
        printf("Falha ao criar servidor TCP\n");
        return -1;
    }

    //vincula um PCB (Protocol Control Block) TCP a um endereço IP e porta específicos.
    if (tcp_bind(server, IP_ADDR_ANY, 80) != ERR_OK)
    {
        printf("Falha ao associar servidor TCP à porta 80\n");
        return -1;
    }

    // Coloca um PCB (Protocol Control Block) TCP em modo de escuta, permitindo que ele aceite conexões de entrada.
    server = tcp_listen(server);

    // Define uma função de callback para aceitar conexões TCP de entrada. É um passo importante na configuração de servidores TCP.
    tcp_accept(server, tcp_server_accept);
    printf("Servidor ouvindo na porta 80\n");

    while (true)
    {   
        if (atualizar_display)
        {
            ssd1306_fill(&ssd, false);

            sprintf(str_status, "Porta %s", estado_porta);
            ssd1306_draw_string(&ssd, str_status, 8, 8);

            if (estado_porta == ABERTA)
            {
                ssd1306_rect(&ssd, 28, 38, 15, 30, true, true);
                ssd1306_rect(&ssd, 28, 68, 15, 30, true, true);
            }
            else
            {
                ssd1306_rect(&ssd, 28, 38, 45, 30, true, true);
            }

            ssd1306_send_data(&ssd);
            atualizar_display = false;
        }

        if (atualizar_matriz)
        {
            if (alarme_on)
            {
                desenharFrame(frameAlarme);
            }
            else if (estado_porta == ABERTA)
            {

                desenharFrame(frameAberto);
            }
            else
            {
                desenharFrame(frameFechado);
            }

            atualizarMatriz();
            atualizar_matriz = false;
        }

        if ((gpio_get(led_green_pin) || gpio_get(led_red_pin)) && !led_callback_on)
        {
            add_alarm_in_ms(1500, led_result_callback, NULL, false);
            led_callback_on = true;
        }

        if (alarme_on && !buzzer_callback_on)
        {   
            add_repeating_timer_ms(300, buzzer_callback, NULL, &timer);
            buzzer_callback_on = true;
        }

        cyw43_arch_poll(); // Necessário para manter o Wi-Fi ativo
        sleep_ms(100);      // Reduz o uso da CPU
    }

    //Desligar a arquitetura CYW43.
    cyw43_arch_deinit();
    return 0;
}

// Inicializar os Pinos GPIO para acionamento dos LEDs da BitDogLab
void inicializarLed(void)
{
    // Configuração dos LEDs como saída
    gpio_init(led_green_pin);
    gpio_set_dir(led_green_pin, GPIO_OUT);
    gpio_put(led_green_pin, false);
    
    gpio_init(led_red_pin);
    gpio_set_dir(led_red_pin, GPIO_OUT);
    gpio_put(led_red_pin, false);
}

void inicializarPwm(uint pino)
{
    // Obter slice e definir pino como PWM
    gpio_set_function(pino, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(pino);

    // Configurar frequência
    pwm_set_wrap(slice, wrap_t);
    pwm_set_clkdiv(slice, clkdiv_t); 
    pwm_set_gpio_level(pino, 0);
}

void limparBuffer(int size, char array[size])
{
    for (int i = 0; i < size; i++)
    {   
        array[i] = '\0';
    }
}

uint8_t escreverBuffer(int size, char array[size], char *values, char break_point)
{   
    for (int i = 0; i < size - 1; i++)
    {
        if (values[i] == break_point)
        {
            return 0;
        }

        array[i] = values[i];
    }

    array[size - 1] = '\0';
    return 1;
}

int64_t led_result_callback(alarm_id_t id, void* user_data)
{
    if (gpio_get(led_green_pin))
    {
        gpio_put(led_green_pin, 0);
    }
    if (gpio_get(led_red_pin))
    {
        gpio_put(led_red_pin, 0);
    }

    led_callback_on = false;
}

bool buzzer_callback(struct repeating_timer *t)
{   
    if (!alarme_on)
    {   
        pwm_set_gpio_level(buzzer_pin, 0);
        buzzer_on = false;
        buzzer_callback_on = false;
        return false;
    }

    if (buzzer_on)
    {
        pwm_set_gpio_level(buzzer_pin, 0);
    }
    else
    {
        pwm_set_gpio_level(buzzer_pin, 500);
    }
    buzzer_on = !buzzer_on;

    return true;
}

// Função de callback ao aceitar conexões TCP
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, tcp_server_recv);
    return ERR_OK;
}

// Tratamento do request do usuário - digite aqui
void user_request(char *request)
{   
    char *recebido;
    char *cursor;

    if (setup)
    {
        recebido = strstr(request, "GET /setup_sent?");
        if (recebido != NULL)
        {
            // Coloca um cursor no início da primeira senha
            cursor = strstr(recebido, "senha1=");
            if (cursor != NULL)
            {
                cursor += strlen("senha1=");

                // Cria e limpa os buffers antes de escrever neles
                limparBuffer(BUFFER1_SIZE, senha);
                limparBuffer(BUFFER2_SIZE, senha_alarme);

                if (escreverBuffer(BUFFER1_SIZE, senha, cursor, '&') == 0)
                {
                    msg_erro = "A senha para abrir a porta possui tamanho invalido";
                    return;
                }

                cursor = strstr(recebido, "senha2=");
                cursor += strlen("senha2="); // Avança para a próxima senha

                if (escreverBuffer(BUFFER2_SIZE, senha_alarme, cursor, ' ') == 0)
                {
                    msg_erro = "A senha para desativar o alarme possui tamanho invalido";
                    return;
                }

                setup = false;
                msg_erro = ""; // Limpa a mensagem de erro se o setup foi bem-sucedido
            }
        }
    }
    else
    {
        recebido = strstr(request, "GET /pass_sent?");
        if (recebido != NULL)
        {   
            // Coloca um cursor no início da senha enviada
            cursor = strstr(recebido, "senha=");
            if (cursor != NULL)
            {   
                cursor += strlen("senha=");

                // Define os parâmetros de acordo com o modo
                const char *senha_definida = alarme_on? senha_alarme : senha;
                uint8_t size = alarme_on? BUFFER2_SIZE : BUFFER1_SIZE;

                // Cria e limpa o buffer antes de escrever nele
                char str_buffer[size];
                limparBuffer(size, str_buffer);

                // Escrever senha digitada no buffer
                if (escreverBuffer(size, str_buffer, cursor, ' ') == 0)
                {
                    msg_erro = "Tamanho de senha invalido";
                    return;
                }

                // Compara a senha com a definida
                if (strcmp(senha_definida, str_buffer) == 0)
                {   
                    if (!alarme_on)
                    {
                        estado_porta = ABERTA;
                        atualizar_display = true;
                    }
                    else
                    {
                        alarme_on = false;
                    }
                    
                    atualizar_matriz = true;
                    tentativas = 0;
                    gpio_put(led_green_pin, 1);
                    msg_erro = "";
                }
                else
                {   
                    if (!alarme_on)
                    {
                        tentativas++;
                        if (tentativas == MAX_TENTATIVAS)
                        {
                            alarme_on = true;
                            atualizar_matriz = true;
                        }
                    }

                    gpio_put(led_red_pin, 1);
                    msg_erro = "Senha incorreta.";
                }
            }
        }
        else if (strstr(request, "GET /close") != NULL)
        {
            estado_porta = FECHADA;
            atualizar_display = true;
            atualizar_matriz = true;
        }
    }
}

// Função de callback para processar requisições HTTP
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (!p)
    {
        tcp_close(tpcb);
        tcp_recv(tpcb, NULL);
        return ERR_OK;
    }

    // Alocação do request na memória dinámica
    char *request = (char *)malloc(p->len + 1);
    memcpy(request, p->payload, p->len);
    request[p->len] = '\0';

    user_request(request);

    // Cria a resposta HTML
    char html[2048];

    if (setup)
    {
        // Instruções html do webserver
        snprintf(html, sizeof(html), // Formatar uma string e armazená-la em um buffer de caracteres
                 "HTTP/1.1 200 OK\r\n"
                 "Content-Type: text/html\r\n"
                 "\r\n"
                 "<!DOCTYPE html>\n"
                 "<html>\n"
                 "<head>\n"
                 "<title>Porta IoT</title>\n"
                 "<style>\n"
                 "body { background-color:rgb(100, 100, 100); font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }\n"
                 "h1 { font-size: 64px; margin-bottom: 30px; }\n"
                 "button { background-color: LightBlue; font-size: 36px; margin: 10px; padding: 20px 40px; border-radius: 10px; }\n"
                 "</style>\n"
                 "</head>\n"
                 "<body>\n"
                 "<h1>Porta IoT</h1>\n"
                 "<p style=\"color: Lightgreen\">Bem vindo! Para iniciar, defina as seguintes senhas:</p>\n"
                 "<form action=\"./setup_sent\">\n"
                 "<label for=\"senha1\">Abrir a porta (%i digitos):</label>\n"
                 "<input type=\"text\" id=\"senha1\" name=\"senha1\" maxlength=%i><br><br>\n"
                 "<label for=\"senha2\">Desligar alarme (%i digitos):</label>\n"
                 "<input type=\"text\" id=\"senha2\" name=\"senha2\" maxlength=%i><br><br>\n"
                 "<input type=\"submit\" value=\"Enviar\">\n"
                 "</form>\n"
                 "<p style=\"color: red;\">%s</p>\n"
                 "</body>\n"
                 "</html>\n", PASSWORD_SIZE, PASSWORD_SIZE, PASSWORD_ALARM_SIZE, PASSWORD_ALARM_SIZE, msg_erro);
    }
    else if (estado_porta == FECHADA)
    {
        msg_alarme = alarme_on? "Alarme acionado. Digite a senha de desativacao!" : "";
        uint8_t curr_pass_size = alarme_on? PASSWORD_ALARM_SIZE : PASSWORD_SIZE;

        // Instruções html do webserver
        snprintf(html, sizeof(html), // Formatar uma string e armazená-la em um buffer de caracteres
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/html\r\n"
                "\r\n"
                "<!DOCTYPE html>\n"
                "<html>\n"
                "<head>\n"
                "<title>Porta IoT</title>\n"
                "<style>\n"
                "body { background-color:rgb(100, 100, 100); font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }\n"
                "h1 { font-size: 64px; margin-bottom: 30px; }\n"
                "button { background-color: LightBlue; font-size: 36px; margin: 10px; padding: 20px 40px; border-radius: 10px; }\n"
                "</style>\n"
                "</head>\n"
                "<body>\n"
                "<h1>Porta IoT</h1>\n"
                "<p>Estado da porta: %s<br><br></p>\n"
                "<p style=\"color: red;\">%s</p>\n"
                "<form action=\"./pass_sent\">\n"
                "<label for=\"senha\">Senha (%i digitos):</label>\n"
                "<input type=\"text\" id=\"senha\" name=\"senha\" maxlength=\"%i\"><br><br>\n"
                "<input type=\"submit\" value=\"Enviar\">\n"
                "<text><br>Tentativas restantes: %i</text>\n"
                "</form>\n"
                "<p style=\"color: red;\">%s</p>\n"
                "</body>\n"
                "</html>\n", estado_porta, msg_alarme, curr_pass_size, curr_pass_size, MAX_TENTATIVAS - tentativas, msg_erro);
    }
    else
    {
        // Instruções html do webserver
        snprintf(html, sizeof(html), // Formatar uma string e armazená-la em um buffer de caracteres
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/html\r\n"
                "\r\n"
                "<!DOCTYPE html>\n"
                "<html>\n"
                "<head>\n"
                "<title>Porta IoT</title>\n"
                "<style>\n"
                "body { background-color:rgb(100, 100, 100); font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }\n"
                "h1 { font-size: 64px; margin-bottom: 30px; }\n"
                "button { background-color: LightBlue; font-size: 36px; margin: 10px; padding: 20px 40px; border-radius: 10px; }\n"
                "</style>\n"
                "</head>\n"
                "<body>\n"
                "<h1>Porta IoT</h1>\n"
                "<p>Estado da porta: %s<br><br></p>\n"
                "<p style=\"color: Lightgreen\">Validado com sucesso! Aperte o botao abaixo quando quiser fechar a porta.</p>\n"
                "<form action=\"./close\"><button>Fechar</button></form>\n"
                "</body>\n"
                "</html>\n", estado_porta);  
    }

    // Escreve dados para envio (mas não os envia imediatamente).
    tcp_write(tpcb, html, strlen(html), TCP_WRITE_FLAG_COPY);

    // Envia a mensagem
    tcp_output(tpcb);

    //libera memória alocada dinamicamente
    free(request);
    
    //libera um buffer de pacote (pbuf) que foi alocado anteriormente
    pbuf_free(p);

    return ERR_OK;
}