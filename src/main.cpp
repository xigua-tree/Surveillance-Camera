#include <cstdio>
#include <csignal>
#include "my_task.h"

static My_Task* g_task = nullptr;

static void signal_handler(int sig)
{
    (void)sig;
    printf("\n[Main] Down\n");
    if (g_task) g_task->stop();
}

int main()
{
    Task_config cfg;

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    My_Task task(cfg);
    g_task = &task;

    int ret = task.start();

    printf("[Main] exit with code %d\n", ret);
    return ret;
}
