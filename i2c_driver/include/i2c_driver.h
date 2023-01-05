/*
 * Nombre de Archivo: i2c.h
 *
 * Descripción: Archivo de Cabecera para driver I2C
 * 				(Contiene funciones, definiciones, estructuras)
 *
 * TECNM - Campus Chihuahua
 *
 *  Arquitectura De Programacion Para El Control de Hardware
 *
 * Integrantes:
 *   José Cruz Armendáriz                   19060756
 *   Luis Octavio Méndez Valles             19060757
 *   Hiram Ochoa Sáenz                      19060760
 *   Manuel Alejandro Quiroz Gallegos       C18061043
 *
 * Docente:
 *   M.C. Alfredo Chacon Aldama
 *
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _DRIVER_I2C_H_
#define _DRIVER_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_types.h>
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "driver/gpio.h"
#include "soc/soc_caps.h"
#include "hal/i2c_types.h"

#define I2C_APB_CLK_FREQ  APB_CLK_FREQ /*!< El Reloj de Origen (Source Clock) I2C es el Reloj APB, 80MHz */

#define I2C_NUM_MAX            (SOC_I2C_NUM) /*!< I2C # Máximo de Puertos*/
#define I2C_NUM_0              (0) /*!< I2C Puerto 0 */
#if SOC_I2C_NUM >= 2
#define I2C_NUM_1              (1) /*!< I2C Puerto 1 */
#endif

// I2C clk flags para uso de los usuarios, puede ampliarse en el futuro.
#define I2C_SCLK_SRC_FLAG_FOR_NOMAL       (0)         /* Se puede elegir cualquier fuente de reloj disponible para la frecuencia especificada.*/
#define I2C_SCLK_SRC_FLAG_AWARE_DFS       (1 << 0)    /* Para REF tick clock, no cambiará con APB.*/
#define I2C_SCLK_SRC_FLAG_LIGHT_SLEEP     (1 << 1)    /* Para el modo de reposo ligero.*/

/**
 * @brief Tamaño mínimo, en bytes, de la estructura privada interna
 * utilizada para describir el enlace de comandos I2C.
 */
#define I2C_INTERNAL_STRUCT_SIZE (24)

/**
 * @brief La siguiente macro se utiliza para determinar el tamaño recomendado del
 * buffer a pasar a la función `i2c_cmd_link_create_static()`.
 *
 * Requiere un parámetro, `TRANSACTIONS`, que describe el número de transacciones
 * que se pretenden realizar en el puerto I2C.
 *
 * Por ejemplo, si uno quiere realizar una lectura en un registro de un dispositivo I2C,
 *  `TRANSACTIONS` debe ser al menos 2, ya que los comandos requeridos son los siguientes:
 *  - write device register
 *  - read register content
 *
 * Señales como "(repeated) start", "stop", "nack", "ack" no se toman en cuenta.
 */
#define I2C_LINK_RECOMMENDED_SIZE(TRANSACTIONS)     (2 * I2C_INTERNAL_STRUCT_SIZE + I2C_INTERNAL_STRUCT_SIZE * \
                                                        (5 * TRANSACTIONS))

												/* Asume que cada transacción del usuario
												 * está rodeada por un "start", la dirección del dispositivo
												 * y una señal "nack/ack".
												 *
												 * Asigna un espacio más para la señal "stop" al final.
												 *
												 * Asignar 2 structuras internas más para las cabeceras.
												 *
												 */

/**
 * @brief I2C - Parámetros de Inicialización
 */
typedef struct{
    i2c_mode_t mode;     /* Modo I2C */
    int sda_io_num;      /* Número GPIO para la señal I2C sda 			*/
    int scl_io_num;      /* Número GPIO para la señal I2C scl 			*/
    bool sda_pullup_en;  /* Modo Pull GPIO interno para señal I2C sda	*/
    bool scl_pullup_en;  /* Modo pull GPIO interno para señal I2C scl.	*/

    union {
        struct {
            uint32_t clk_speed;      /* I2C Frecuencia de Reloj para el Modo Master,
             	 	 	 	 	 	 	 (No  superior a 1MHz por ahora) 				*/
        } master;
        struct {					 /* Configuración del Master */
            uint8_t addr_10bit_en;   /* I2C Habilitación del Modo de Dirección de 10 bits para el modo slave */
            uint16_t slave_addr;     /* I2C dirección para el Modo slave */
            uint32_t maximum_speed;  /* I2C velocidad de Reloj Esperada de SCL */
        } slave;                     /* I2C Configuración del Slave */
    };
    uint32_t clk_flags;              /* Operación bit a bit (Bitwise) de ``I2C_SCLK_SRC_FLAG_**FOR_DFS**`` para la elección de la fuente CLK (clk source)*/
} i2c_config_t;


typedef void *i2c_cmd_handle_t;    /* Manejador de comandos I2C  */

/*----------------------------------------------------------------------------------------------------------|//
|---- 								Funciones del Driver I2C 										    ----|//
|-----------------------------------------------------------------------------------------------------------|*/

void first_message (void);

void second_message (void);

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear (void);

esp_err_t set_i2c_lcd(void);

esp_err_t set_i2c(bool MS_SELECT);
/*
	* @brief Configura el modulo I2C
	*
	* @return
	*     - ESP_OK   				-> Correcto.
	*/

esp_err_t i2c_instalacion_driver(i2c_port_t i2c_num, i2c_mode_t mode, size_t slv_rx_buf_len, size_t slv_tx_buf_len, int intr_alloc_flags);
   /*
	* @brief Instalar un controlador I2C
	*
	* @param i2c_num			-> Número de puerto I2C
	* @param Modo I2C 			-> (Master o Slave)
	* @param slv_rx_buf_len 	-> Tamaño del buffer de recepción. Sólo el modo esclavo utilizará este valor, se ignora en el modo maestro.
	* @param slv_tx_buf_len	-> Tamaño del buffer de envío. Sólo el modo esclavo utilizará este valor, se ignora en el modo maestro.
	* @param intr_alloc_flags 	-> Banderas utilizadas para asignar la interrupción. Uno o varios (ORred) valores ESP_INTR_FLAG_*.
	*                          	(Para más información, véase en esp_intr_alloc.h).
	*        @Nota
	*        En modo maestro, si es probable que la caché esté deshabilitada (como flash de escritura) y el esclavo es sensible al tiempo,
	*        Se sugiere utilizar `ESP_INTR_FLAG_IRAM`. En este caso, utilice la memoria asignada desde la RAM interna en la función de lectura y escritura i2c,
	*        porque no podemos acceder a la psram (si el psram está habilitado) en la función de manejo de interrupciones cuando la caché está desactivada.
	*
	* @return
	*     - ESP_OK   				-> Correcto.
	*     - ESP_ERR_INVALID_ARG	-> Error de parámetro.
	*     - ESP_FAIL 				-> Error de instalación del controlador.
	*/

esp_err_t i2c_configuracion_parametros(i2c_port_t i2c_num, const i2c_config_t *i2c_conf);

	/*
	 * @brief Configura un bus I2C con la configuración dada.
	 *
	 * @param i2c_num 				-> Puerto I2C para configurarlo
	 * @param i2c_conf 				-> Puntero a la configuración I2C
	 *
	 * @return
	 *     - ESP_OK   				-> Correcto.
	 *     - ESP_ERR_INVALID_ARG 	-> Error de parámetro.
	 */

esp_err_t i2c_reset_tx_queue(i2c_port_t i2c_num);

	/*
	 * @brief Reinicia I2C TX Hardware fifo (First In, First Out)
	 *
	 * @param i2c_num 				-> Número de Puerto I2C
	 *
	 * @return
	 *     - ESP_OK   				-> Correcto.
	 *     - ESP_ERR_INVALID_ARG 	-> Error de parámetro.
	 */

esp_err_t i2c_reset_rx_queue(i2c_port_t i2c_num);
	/*
	 * @brief Reinicia I2C RX Hardware fifo (First In, First Out)
	 *
	 * @param i2c_num 				-> Número de Puerto I2C
	 *
	 * @return
	 *     - ESP_OK   				-> Correcto.
	 *     - ESP_ERR_INVALID_ARG 	-> Error de parámetro.
	 */

esp_err_t i2c_interrup_register(i2c_port_t i2c_num, void (*fn)(void *), void *arg,
										int intr_alloc_flags, intr_handle_t *handle);
	/*
	 * @brief Registra un controlador (handler) de ISR del I2C.
	 *
	 * @param i2c_num 				-> Número de Puerto I2C al que conectar el controlador.
	 * @param fn 					-> Función ISR del controlador.
	 * @param arg 					-> Parámetro para el controlador ISR.
	 * @param intr_alloc_flags 		-> Banderas utilizadas para asignar la interrupción.
	 * 								   Uno o varios (ORred) ESP_INTR_FLAG_* valores.
	 * 								   (Para más información, véase en esp_intr_alloc.h)
	 *
	 * @param handle 				-> Maneja el retorno de esp_intr_alloc.
	 *
	 * @return
	 *     - ESP_OK   				-> Correcto.
	 *     - ESP_ERR_INVALID_ARG 	-> Error de parámetro.
	 */

esp_err_t i2c_gpio_pin_config(i2c_port_t i2c_num, int sda_io_num, int scl_io_num,
                      	  	  bool sda_pullup_en, bool scl_pullup_en, i2c_mode_t mode);
	/*
	 * @brief Configura los pines GPIO para las señales SCL y SDA de I2C.
	 *
	 * @param i2c_num 				-> Número de Puerto I2C
	 * @param sda_io_num 			-> Número GPIO para la señal I2C SDA
	 * @param scl_io_num 			-> Número GPIO para la señal I2C SCL
	 * @param sda_pullup_en 		-> Habilita el pullup interno para el pin SDA
	 * @param scl_pullup_en 		-> Habilita el pullup interno para el pin SCL
	 * @param mode 					-> Modo I2C (Master o Slave)
	 *
	 * @return
	 *     - ESP_OK   				-> Correcto.
	 *     - ESP_ERR_INVALID_ARG 	-> Error de parámetro.
	 */

esp_err_t i2c_master_write_to_device(i2c_port_t i2c_num, uint8_t device_address,
                                     const uint8_t* write_buffer, size_t write_size,
                                     TickType_t ticks_to_wait);

esp_err_t i2c_master_escritura_lectura_device(i2c_port_t i2c_num, uint8_t device_address,
                                       const uint8_t* write_buffer, size_t write_size,
                                       uint8_t* read_buffer, size_t read_size,
                                       TickType_t ticks_to_wait);

	/*
	 * @brief 	Realiza una escritura en un dispositivo conectado a un puerto I2C determinado.
	 *
	 *			Esta función contiene a `i2c_master_start()`, `i2c_master_write()`, `i2c_master_read()`, etc.
	 *        	Sólo puede ser llamada en modo Master I2C.
	 *
	 * @param i2c_num 				-> Número de puerto I2C en el que se realizará la transferencia
	 * @param device_address 		-> Dirección de 7 bits del dispositivo I2C
	 * @param write_buffer 			-> Bytes a enviar en el bus
	 * @param write_size 			-> Tamaño, en bytes, del búfer de escritura
	 * @param ticks_to_wait 		-> Ticks máximos a esperar antes de emitir un timeout.
	 *
	 * @return
	 *     - ESP_OK   				-> Correcto.
	 *     - ESP_ERR_INVALID_ARG 	-> Error de parámetro.
	 *     - ESP_FAIL 				-> Error de comando de envío, el esclavo no ha ACK la transferencia.
	 *     - ESP_ERR_INVALID_STATE	-> El controlador I2C no está instalado o no está en modo maestro.
	 *     - ESP_ERR_TIMEOUT 		-> Tiempo de espera de la operación porque el Bus está ocupado.
	 */

i2c_cmd_handle_t i2c_cmd_link_create_static(uint8_t* buffer, uint32_t size);

	/*
	 * @brief Crea e inicializa una lista de comandos I2C con un buffer dado.
	 *        Todas las asignaciones de datos o señales (START, STOP, ACK, ...)
	 *        se realizarán dentro de este buffer.
	 *
	 *        Este buffer debe ser válido durante toda la transacción.
	 *        Después de terminar las transacciones I2C, se requiere llamar a `i2c_cmd_link_delete_static()`.
	 *
	 * @note Se recomienda encarecidamente no asignar este búfer a la pila. El tamaño de los datos
	 *       usados debajo puede aumentar en el futuro, resultando en un posible desbordamiento
	 *       de la pila ya que la macro `I2C_LINK_RECOMMENDED_SIZE` también devolvería un valor mayor.
	 *
	 *       Una mejor opción es utilizar un buffer asignado estática o dinámicamente (con `malloc`).
	 *
	 * @param buffer 				-> Buffer a utilizar para la asignación de comandos
	 * @param size 					-> Tamaño en bytes del búfer
	 *
	 * @return Controla el enlace de comandos I2C o NULL si el buffer proporcionado es demasiado pequeño, por favor
	 *         utiliza la macro `I2C_LINK_RECOMMENDED_SIZE` para obtener el tamaño recomendado para el buffer.
	 */
void i2c_cmd_link_delete_static(i2c_cmd_handle_t cmd_handle);
	/*
	 * @brief Libera la lista de comandos I2C asignada estáticamente con `i2c_cmd_link_create_static`.
	 *
	 * @param cmd_handle 			-> Lista de comandos I2C asignados estáticamente. Este Controlador
	 * 									debe ser creado gracias a la función `i2c_cmd_link_create_static()`.
	 */

esp_err_t i2c_master_start(i2c_cmd_handle_t cmd_handle);
	/*
	 * @brief Pone en cola una "señal START" a la lista de comandos dada.
	 *        Esta función sólo se llamará en modo maestro I2C.
	 *        Llama a `i2c_master_cmd_begin()` para enviar todos los comandos en cola.
	 *
	 * @param cmd_handle 			-> Lista de comandos I2C
	 *
	 * @return
	 *     - ESP_OK   				-> Correcto.
	 *     - ESP_ERR_INVALID_ARG 	-> Error de parámetro.
	 *     - ESP_ERR_NO_MEM 		-> El buffer estático utilizado para crear `cmd_handler` es demasiado pequeño.
	 *     - ESP_FAIL 				-> No queda memoria en el heap.
	 */

esp_err_t i2c_master_write_byte(i2c_cmd_handle_t cmd_handle, uint8_t data, bool ack_en);
	/*
	 * @brief Pone en cola un comando "escribir byte" en la lista de comandos.
	 *        Se enviará un único byte por el puerto I2C. Esta función sólo se llama en modo maestro I2C.
	 *        Llama a `i2c_master_cmd_begin()` para enviar todos los comandos en cola
	 *
	 * @param cmd_handle 			-> Lista de comandos I2C
	 * @param data 					-> Byte a enviar por el puerto
	 * @param ack_en 				-> Activar señal ACK
	 *
	 * @return
	 *     - ESP_OK   				-> Correcto.
	 *     - ESP_ERR_INVALID_ARG 	-> Error de parámetro.
	 *     - ESP_ERR_NO_MEM 		-> El buffer estático utilizado para crear `cmd_handler` es demasiado pequeño.
	 *     - ESP_FAIL 				-> No queda memoria en el heap.
	 */

esp_err_t i2c_master_write(i2c_cmd_handle_t cmd_handle, const uint8_t *data, size_t data_len, bool ack_en);
	/*
	 * @brief Pone en cola un comando "escribir (múltiples) bytes" a la lista de comandos.
	 *        Esta función sólo debe ser llamada en modo maestro I2C.
	 *        Llamar a `i2c_master_cmd_begin()` para enviar todos los comandos en cola.
	 *
	 * @param cmd_handle 			-> Lista de comandos I2C
	 * @param data 					-> Bytes a enviar. Este búfer permanecerá **válido** hasta que finalice la transacción.
	 *
	 * @param ack_en 				-> Activar señal ACK
	 * @param data 					-> Bytes to send. This buffer shall remain **valid** until the transaction is finished.
	 *             					   Si la PSRAM está habilitada y `intr_flag` está ajustado a `ESP_INTR_FLAG_IRAM`,
	 *             					   `data` debe ser asignada desde la RAM interna.
	 * @param data_len 				-> Longitud, en bytes, del búfer de datos.
	 * @param ack_en 				-> Activar señal ACK
	 *
	 * @return
	 *     - ESP_OK   				-> Correcto.
	 *     - ESP_ERR_INVALID_ARG 	-> Error de parámetro.
	 *     - ESP_ERR_NO_MEM 		-> El buffer estático utilizado para crear `cmd_handler` es demasiado pequeño.
	 *     - ESP_FAIL 				-> No queda memoria en el heap.
	 */

esp_err_t i2c_master_lee_byte(i2c_cmd_handle_t cmd_handle, uint8_t *data, i2c_ack_type_t ack);
	/*
	 * @brief Pone en cola un comando "leer byte" en la lista de comandos.
	 *        Se leerá un único byte en el bus I2C. Esta función sólo se llamará en modo maestro I2C.
	 *        Llama a `i2c_master_cmd_begin()` para enviar todos los comandos en cola
	 *
	 * @param cmd_handle 			-> Lista de comandos I2C
	 * @param data 					-> Puntero donde se almacenará el byte recibido. Este búfer permanecerá **válido** hasta que finalice la transacción.
	 * @param ack 					-> Señal ACK
	 *
	 * @return
	 *     - ESP_OK   				-> Correcto.
	 *     - ESP_ERR_INVALID_ARG 	-> Error de parámetro.
	 *     - ESP_ERR_NO_MEM 		-> El buffer estático utilizado para crear `cmd_handler` es demasiado pequeño.
	 *     - ESP_FAIL 				-> No queda memoria en el heap.
	 */

esp_err_t i2c_master_read(i2c_cmd_handle_t cmd_handle, uint8_t *data, size_t data_len, i2c_ack_type_t ack);
	/*
	 * @brief Pone en cola un comando "leer (multiples) bytes" en la lista de comandos.
	 *        Se leerán múltiples bytes en el bus I2C. Esta función sólo se llamará en modo Master I2C.
	 *        Llama a `i2c_master_cmd_begin()` para enviar todos los comandos en cola.
	 *
	 * @param cmd_handle 			-> Lista de comandos I2C.
	 * @param data 					-> Puntero donde se almacenará el byte recibido. Este búfer permanecerá **válido** hasta que finalice la transacción.
	 * @param data_len				-> amaño, en bytes, del búfer `data`.
	 * @param ack 					-> Señal ACK.

	 *
	 * @return
	 *     - ESP_OK   				-> Correcto.
	 *     - ESP_ERR_INVALID_ARG 	-> Error de parámetro.
	 *     - ESP_ERR_NO_MEM 		-> El buffer estático utilizado para crear `cmd_handler` es demasiado pequeño.
	 *     - ESP_FAIL 				-> No queda memoria en el heap.
	 */

esp_err_t i2c_master_stop(i2c_cmd_handle_t cmd_handle);
	/*
	 * @brief Pone en cola una "señal de STOP" a la lista de comandos dada.  Esta función sólo se llamará en modo Master I2C.
	 *        Llama a `i2c_master_cmd_begin()` para enviar todos los comandos en cola.
	 *
	 * @param cmd_handle 			-> Lista de comandos I2C.
	 *
	 * @return
	 *     - ESP_OK   				-> Correcto.
	 *     - ESP_ERR_INVALID_ARG 	-> Error de parámetro.
	 *     - ESP_ERR_NO_MEM 		-> El buffer estático utilizado para crear `cmd_handler` es demasiado pequeño.
	 *     - ESP_FAIL 				-> No queda memoria en el heap.
	 */

esp_err_t i2c_master_cmd_begin(i2c_port_t i2c_num, i2c_cmd_handle_t cmd_handle, TickType_t ticks_to_wait);
	/*
	 * @brief Envía todos los comandos en cola al bus I2C, en modo maestro.
	 *        La tarea se bloqueará hasta que se hayan enviado todos los comandos.
	 *        El puerto I2C está protegido por mutex, por lo que esta función es thread-safe.
	 *        Esta función sólo se llamará en modo maestro I2C.
	 *
	 * @param i2c_num 				-> Número de puerto I2C
	 * @param cmd_handle 			-> Lista de comandos I2C.
	 * @param ticks_to_wait 		-> Ticks máximos a esperar antes de emitir un timeout.
	 *
	 * @return
	 *     - ESP_OK   				-> Correcto.
	 *     - ESP_ERR_INVALID_ARG 	-> Error de parámetro.
	 *     - ESP_FAIL 				-> Error de comando de envío, el esclavo no ha ACK la transferencia.
	 *     - ESP_ERR_INVALID_STATE 	-> El controlador I2C no está instalado o no está en modo maestro.
	 *     - ESP_ERR_TIMEOUT 		-> Tiempo de espera de la operación porque el bus está ocupado (timeout).
	 */

int i2c_slave_escribe_buffer(i2c_port_t i2c_num, const uint8_t *data, int size, TickType_t ticks_to_wait);
	/*
	 * @brief Escribe bytes en el ringbuffer interno de datos del esclavo I2C. Cuando el TX fifo esté vacío, el ISR
	 *        llenar el hardware FIFO con los datos del ringbuffer interno.
	 *        @nota Esta función sólo se llamará en modo esclavo I2C.
	 *
	 * @param i2c_num 				-> Número de puerto I2C
	 * @param data 					-> Bytes a escribir en el buffer interno
	 * @param size 					-> Tamaño, en bytes, del búfer `data`
	 * @param ticks_to_wait 		-> Máximo de ticks a esperar.
	 *
	 * @return
	 *     - ESP_FAIL 				-> (-1)  Error de parámetro.
	 *     - Other 					-> (>=0) Número de bytes de datos enviados al búfer del esclavo I2C.
	 */

int i2c_slave_lee_buffer(i2c_port_t i2c_num, uint8_t *data, size_t max_size, TickType_t ticks_to_wait);
	/*
	 * @brief Leer bytes del buffer interno I2C. Cuando el bus I2C reciba datos, el ISR los copiará
	 *        desde el hardware RX FIFO al ringbuffer interno.
	 *        Al llamar a esta función se copiarán los bytes del ringbuffer interno al buffer de usuario `data`.
	 *        @nota Esta función sólo se llamará en modo esclavo I2C.
	 *
	 * @param i2c_num 				-> Número de puerto I2C
	 * @param data 					-> Buffer a llenar con los bytes del ringbuffer
	 * @param max_size 				-> Máximo de bytes a leer
	 * @param ticks_to_wait 		-> Máximo de ticks a esperar.
	 *
	 * @return
	 *     - ESP_FAIL 				-> (-1)  Error de parámetro.
	 *     - Other 					-> (>=0) Número de bytes de datos enviados al búfer del esclavo I2C.
	 */

#ifdef __cplusplus
}
#endif

#endif /*_DRIVER_I2C_H_*/
