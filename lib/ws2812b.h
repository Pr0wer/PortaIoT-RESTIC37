#ifndef MATRIZ
#define MATRIZ

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812b.pio.h"

// Tamanho da matriz de LEDs
#define MATRIZ_ROWS 5
#define MATRIZ_COLS 5
#define MATRIZ_SIZE MATRIZ_ROWS * MATRIZ_COLS

// Pino
const uint matriz_pin = 7;

// Estrutura para armazenar um pixel com valores RGB
typedef struct Rgb
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} Rgb;

// Buffer para matriz de LEDs
static Rgb matriz[MATRIZ_SIZE];

// Variáveis para configuração do PIO
PIO np_pio;
uint sm;

// Altera a cor de um LED no buffer da matriz
static void alterarLed(int i, uint8_t red, uint8_t green, uint8_t blue)
{
  matriz[i].r = red;
  matriz[i].g = green;
  matriz[i].b = blue;
}

// Atualiza a matriz de LEDs com o conteúdo do buffer
void atualizarMatriz()
{
  for (int i = 0; i < MATRIZ_SIZE; i++)
  {
    pio_sm_put_blocking(np_pio, sm, matriz[i].g);
    pio_sm_put_blocking(np_pio, sm, matriz[i].r);
    pio_sm_put_blocking(np_pio, sm, matriz[i].b);

  }
  sleep_us(100); // Espera 100us para a próxima mudança, de acordo com o datasheet
}

// Limpa buffer e atualiza a matriz
void limparMatriz()
{
   for (uint i = 0; i < MATRIZ_SIZE; i++) 
   {
     alterarLed(i, 0, 0, 0);
   }
}

// Inicializa a matriz de LED WS2812B. Baseado no exemplo neopixel_pio do repo BitDogLab
void inicializarMatriz()
{
  // Cria PIO
  uint offset = pio_add_program(pio0, &ws2812b_program);
  np_pio = pio0;

  // Pede uma máquina de estado do PIO
  sm = pio_claim_unused_sm(np_pio, false);
  if (sm < 0) 
  {
    np_pio = pio1;
    sm = pio_claim_unused_sm(np_pio, true);
  }

  // Roda programa na máquina de estado
  ws2812b_program_init(np_pio, sm, offset, matriz_pin, 800000.f);

  limparMatriz();
  atualizarMatriz();
}

// Converte a posição de um array bidimensional para a posição correspondente na matriz de LED
static uint obterPosicao(uint linha, uint coluna)
{ 
  // Inverter linha
  uint linhaMatriz = MATRIZ_ROWS - 1 - linha;

  uint colunaMatriz;
  if (linha % 2 == 0)
  {
    // Se a linha for par, a informação é enviada da direita para a esquerda (inversão da coluna)
    colunaMatriz = MATRIZ_COLS - 1 - coluna;
  }
  else
  {
    // Senão, é enviada da esquerda para a direita (coluna é a mesma)
    colunaMatriz = coluna;
  }

  // Converter novo indíce de array bidimensional para unidimensional e retornar valor
  return MATRIZ_ROWS * linhaMatriz + colunaMatriz;
}

// Desenha n LEDs em uma coluna na matriz dado um ponto inicial (de cima para baixo)
void desenharColuna(uint pontoInit, uint coluna, uint n, Rgb cor)
{
  for (int l = pontoInit; l < pontoInit + n; l++)
  {
    // Obter indíce correto de acordo com o modo de envio de informação do WS2812B e adicionar pixel correspondente ao buffer
    uint index = obterPosicao(l, coluna);
    alterarLed(index, cor.r, cor.g, cor.b);
  }
}

// Desenha um frame MATRIZ_ROWSxMATRIZ_COLS no buffer da matriz de LED
void desenharFrame(Rgb frame[MATRIZ_ROWS][MATRIZ_COLS])
{
  // Iterar por cada pixel
  for (int linha = 0; linha < MATRIZ_ROWS; linha++)
  {
    for (int coluna = 0; coluna < MATRIZ_COLS; coluna++)
    { 
      // Obter indíce correto de acordo com o modo de envio de informação do WS2812B e adicionar pixel correspondente ao buffer
      uint index = obterPosicao(linha, coluna);
      alterarLed(index, frame[linha][coluna].r, frame[linha][coluna].g, frame[linha][coluna].b);
    }
  }
}

#endif