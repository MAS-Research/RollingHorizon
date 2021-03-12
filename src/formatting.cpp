/*
 * The MIT License
 *
 * Copyright 2018 Matthew Zalesak.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "formatting.hpp"

#include <chrono>
#include <sstream>

int decode_time(int time)
{
    int hour = (time / 10000) % 100;
    int minute = (time / 100) % 100;
    int second = (time / 1) % 100;
    
    return 3600*hour + 60*minute + second;
}

int encode_time(int time)
{
    int hour = (time / 3600);
    int minute = (time / 60) % 60;
    int second = (time / 1) % 60;
    
    return 10000*hour + 100*minute + second;
}


void info(std::string s, Color color)
{
    std::cout << "[INFO] \033[;3";
    std::cout << color;
    std::cout << "m ";
    std::cout << s;
    std::cout << "\033[0m" << std::endl;
}

int read_time(std::string const s)
{
    std::string flat_string;
    std::string token;
    std::istringstream ss(s);
    while (getline(ss, token, ':'))
    {
        if (token.size() == 1)
            flat_string += "0";
        flat_string += token;
    }
    return decode_time(stoi(flat_string));
}

std::string current_time()
{
    time_t current_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string time_string = ctime(&current_time);
    time_string.pop_back();
    return time_string;
}
