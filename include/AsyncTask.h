#ifndef _ASYNC_H_
#define _ASYNC_H_

typedef enum {
    UNDEFINED,
    CALIBRATE_RANGE,
    CALIBRATE_ANGLE,
    READ_NORTH,
    POINT_NORTH,
    CONNECT,
    WAIT
} status_t; 

typedef void (*TaskFunc_t) (const unsigned long&);

class AsyncTask {
public:
    AsyncTask(const status_t& task, const status_t& nextTask, const unsigned long& taskDuration
              , TaskFunc_t start
              , TaskFunc_t loop
              , TaskFunc_t end
             );

    status_t taskLoop(const status_t& task, const unsigned long& currentTime);

    unsigned long taskStart;

    TaskFunc_t start;
    TaskFunc_t loop;
    TaskFunc_t end;

    const status_t task;
    const status_t nextTask;
    const unsigned long taskDuration;
};

#endif
