/* =========================================================================
 * Unity Project - A Test Framework for C
 * Copyright (c) 2007-21 Mike Karlesky, Mark VanderVoord, Greg Williams
 * [Released under MIT License]
 * ========================================================================= */

#ifndef UNITY_FRAMEWORK_H
#define UNITY_FRAMEWORK_H

#include <stdio.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define UNITY_TEST_ASSERT(condition, line, message) \
    if (!(condition)) { UnityTestFail(line, message); return; }

#define TEST_ASSERT(condition) \
    UNITY_TEST_ASSERT((condition), __LINE__, "Expression evaluated to false: " #condition)

#define TEST_ASSERT_TRUE(condition) \
    UNITY_TEST_ASSERT((condition), __LINE__, "Expected TRUE Was FALSE")

#define TEST_ASSERT_FALSE(condition) \
    UNITY_TEST_ASSERT(!(condition), __LINE__, "Expected FALSE Was TRUE")

#define TEST_ASSERT_EQUAL_INT(expected, actual) \
    UNITY_TEST_ASSERT(((int)(expected) == (int)(actual)), __LINE__, "Integer values not equal")

#define TEST_ASSERT_EQUAL_HEX8(expected, actual) \
    UNITY_TEST_ASSERT(((uint8_t)(expected) == (uint8_t)(actual)), __LINE__, "Hex8 values not equal")

#define TEST_ASSERT_EQUAL_HEX16(expected, actual) \
    UNITY_TEST_ASSERT(((uint16_t)(expected) == (uint16_t)(actual)), __LINE__, "Hex16 values not equal")

#define TEST_ASSERT_EQUAL_HEX32(expected, actual) \
    UNITY_TEST_ASSERT(((uint32_t)(expected) == (uint32_t)(actual)), __LINE__, "Hex32 values not equal")

#define TEST_ASSERT_EQUAL_MEMORY(expected, actual, len) \
    UNITY_TEST_ASSERT((memcmp((expected), (actual), (len)) == 0), __LINE__, "Memory buffers mismatch")

#define RUN_TEST(func) \
    UnityRunTest(func, #func, __LINE__)

typedef struct {
    uint32_t NumberOfTests;
    uint32_t TestFailures;
    uint32_t TestIgnores;
    const char *CurrentTestName;
    uint32_t CurrentTestLineNumber;
} Unity_t;

extern Unity_t Unity;

void UnityBegin(const char *filename);
int  UnityEnd(void);
void UnityTestFail(uint32_t line, const char *message);
void UnityRunTest(void (*testFunc)(void), const char *name, uint32_t line);

void setUp(void);
void tearDown(void);

#endif /* UNITY_FRAMEWORK_H */
