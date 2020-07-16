#ifndef MICROPLANIFICADOR_H_
#define MICROPLANIFICADOR_H_
#include <stdint.h>
typedef struct
{
	int bcet;
	int wcet;
	int et;
	int et_wcet;
	int offset;
	int period;
	void (*task)(void *p);
	void *param;
} TaskStat;

void inicializar_despachador(	TaskStat *lista,
								uint32_t len,
								void (*start_timer)(void),
								uint32_t (*stop_timer)(void),
								void (*falla_sistema)(void));

int agregar_tarea(TaskStat *lista,
				  void (*tarea)(void *p),
				  void *param,
				  int offset,
				  int periodo,
				  int bcet,
				  int wcet);

void despachar_tareas(void);

#endif /* MICROPLANIFICADOR_H_ */
