#include "AsyncTask.h"

AsyncTask::AsyncTask(const status_t& task, const status_t& nextTask, const unsigned long& taskDuration, 
                     TaskFunc_t start, TaskFunc_t loop, TaskFunc_t end)
    : taskStart(0)
    , start(start)
    , loop(loop)
    , end(end)
    , task(task)
    , nextTask(nextTask)
    , taskDuration(taskDuration)
{}

status_t AsyncTask::taskLoop(const status_t& task, const unsigned long& currentTime) {
    if ( task != this->task ) {
        return task;
    } else if ( taskStart == 0 ) {
        taskStart = currentTime;
        start(currentTime);
        return task;
    } else if ( currentTime - taskStart >= taskDuration ) {
        end(currentTime);
        taskStart = 0;
        return nextTask;
    } else {
        loop(currentTime);
        return task;
    }
}
