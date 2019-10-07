#ifndef __TASK_HH__
#define __TASK_HH__

class Task{
    private:
        Task* next;
        Task* nextWaiting;
        void (*Task_PC)();
        unsigned long TaskSleep;
        unsigned long TaskHitCount;
        unsigned short priority;
        unsigned char TaskStatus;
    public:
        Task(
            void            (*main)(),
            unsigned long   userStackSize,
            unsigned short  queueSIze,
            unsigned short  priority,
            const char*     taksName
        );
        static const char* const MyName()
        { return currTask->name; };
        static unsigned short MyPriority()
        { return currTask->priority; };
        static Task* Current()
        { return currTask; };
        Task* Next() const
        { return next; };
        void Start()
        {TaskStatus &= ~STARTED; };
        static void Dsched()
        { asm("TRAP #1"); };

        enum{
            RUN             = 0x00,
            BLKD            = 0x01,
            STARTED         = 0x02,
            TERMINATED      = 0x04,
            SLEEP           = 0x08,
            FAILED          = 0x10
        };
    private:
        Task();
        ~Task();
        static int          SchedulerStarted;
        static Task*        currTask;

        char*               Stack;
        const char*         name;
        int                 ExitCode;
};

#endif __TASK_HH__