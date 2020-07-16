#include "miniplanificador.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include <string.h>

/*
 * Punteros a función de todo lo que es sistema dependiente
 * o que el planificador no puede decidir.
 */
static void (*fallar_sistema)(void);
static void (*monitor_start)(void);
static uint32_t (*monitor_stop)(void);

/*
 * Variables para gestionar el planificador
 */
static uint32_t max_len_lista_tareas;
static uint32_t tareas_a_despachar;
static TaskStat *lista_tareas;

void inicializar_despachador(TaskStat *lista, uint32_t len,
		void (*start_timer)(void), uint32_t (*stop_timer)(void),
		void (*falla_sistema)(void))
{
	uint32_t i;
	monitor_start = start_timer;
	monitor_stop = stop_timer;
	fallar_sistema = falla_sistema;
	tareas_a_despachar = 0;
	max_len_lista_tareas = len;
	lista_tareas = lista;
	for (i = 0; i < len; i++)
		memset(&lista[i], 0, sizeof(TaskStat));
}

int agregar_tarea(TaskStat *lista, void (*tarea)(void *p), void *param,
		int offset, int periodo, int bcet, int wcet)
{
	// Chequeos
	if ((periodo == 0) || (tarea == NULL)
			|| tareas_a_despachar == max_len_lista_tareas)
		return -1;

	//Punteros de la tarea.
	lista[tareas_a_despachar].task = tarea;
	lista[tareas_a_despachar].param = param;

	//Periodo y offset
	lista[tareas_a_despachar].period = periodo;
	lista[tareas_a_despachar].offset = offset;

	//Cargo los tiempos.
	lista[tareas_a_despachar].et = 0;
	lista[tareas_a_despachar].bcet = bcet;
	lista[tareas_a_despachar].wcet = wcet;
	tareas_a_despachar++;
	return 0;
}

int despachar_tarea(TaskStat *estado)
{
	int ret = 0;
	int valor_us;
	if (!estado->offset)
	{
		estado->offset = estado->period - 1;
		monitor_start();
		estado->task(estado->param);
		valor_us = monitor_stop();
		estado->et = valor_us;
		if (valor_us < estado->bcet || valor_us > estado->wcet)
			ret--;
		if (estado->et_wcet < estado->et)
			estado->et_wcet = estado->et;
	}
	else
	{
		estado->offset--;
	}
	return ret;
}

void despachar_tareas(void)
{
	uint32_t i;
	int fallar = 0;
	for (i = 0; i < tareas_a_despachar; i++)
	{
		fallar = despachar_tarea(&lista_tareas[i]);
		if (fallar)
		{
			break;
		}
	}
	if (fallar)
	{
		fallar_sistema();
	}
}
