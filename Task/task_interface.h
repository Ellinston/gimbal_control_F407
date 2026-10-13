#ifndef TASK_INTERFACE_H
#define TASK_INTERFACE_H

typedef struct
{
    void (*Init)(void);
    void (*Update)(void); /* 主循环调用；任务内部按 10ms 执行控制。 */
} Task_Interface_t;

#endif
