#ifndef LOGGER_H
#define LOGGER_H

#include <cstdio>
#include <iostream>
#include <string>

class Logger {
public:
    Logger(std::string filename) {
        log = fopen(filename.c_str(), "w");
        if (log == NULL) std::cout << filename << " can't be opened!";};
    virtual ~Logger() {if (log != NULL) fclose(log);};

    std::FILE *log;
};

#endif
