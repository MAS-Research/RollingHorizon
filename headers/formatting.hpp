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

#ifndef FORMATTING_HPP
#define FORMATTING_HPP

#include<iostream>
#include<string>

/* Take time as HHMMSS to integer. */
int decode_time(int time);

/* Take time as integer to HHMMSS. */
int encode_time(int time);

/* Take a time in the format (h)h:mm:ss and return it as a number of seconds. */
int read_time(std::string s);

/* Return string representing the current system time. */
std::string current_time();

enum Color { Black, Red, Green, Yellow, Blue, Purple, Cyan, White };

void info(std::string const s, Color color);

#endif // FORMATTING_HPP
