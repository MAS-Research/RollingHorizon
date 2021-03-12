/*
 * The MIT License
 *
 * Copyright 2018 Vindula Jayawardana and Matthew Zalesak.
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

#include "threads.hpp"

#include <math.h>

Threads::Threads(int thread_count)  :
        thread_count (thread_count),
        pool (thpool_init(thread_count))
{}


void Threads::auto_thread(int job_count, void (*function)(void*), void* data)
{
    double jobs_per_thread = job_count / (double) thread_count;
    struct thread_data t[thread_count];
    for (auto i = 0; i < thread_count; i++)
    {
        int start = ceil(i * jobs_per_thread);
        int end = ceil((i + 1) * jobs_per_thread);
        if (end > job_count)
            end = job_count;
        t[i] = {start, end, data};
        thpool_add_work(pool, function, (void*) &t[i]);
    }
    
    thpool_wait(pool);
}

void Threads::mega_thread(int job_count, void (*function)(void*), void* data)
{
    struct thread_data t[job_count];
    for (auto i = 0; i < job_count; i++)
    {
        t[i] = {i, i + 1, data};
        thpool_add_work(pool, function, (void*) &t[i]);
    }
    
    thpool_wait(pool);
}

