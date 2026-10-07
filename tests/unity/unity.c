/* =========================================================================
 * Unity Project - A Test Framework for C
 * Copyright (c) 2007-21 Mike Karlesky, Mark VanderVoord, Greg Williams
 * [Released under MIT License]
 * ========================================================================= */

#include "unity.h"

Unity_t Unity;

void UnityBegin(const char *filename)
{
    Unity.NumberOfTests = 0;
    Unity.TestFailures = 0;
    Unity.TestIgnores = 0;
    Unity.CurrentTestName = NULL;
    Unity.CurrentTestLineNumber = 0;
    printf("\n------------------------------------------------------------\n");
    printf("   UNITY UNIT TEST EXECUTION: %s\n", filename);
    printf("------------------------------------------------------------\n");
}

int UnityEnd(void)
{
    printf("\n============================================================\n");
    printf("   TEST SUMMARY: %u Tests, %u Failures, %u Ignored\n",
           Unity.NumberOfTests, Unity.TestFailures, Unity.TestIgnores);
    if (Unity.TestFailures == 0) {
        printf("   RESULT: [PASS] (100%% Tests Verified)\n");
    } else {
        printf("   RESULT: [FAIL]\n");
    }
    printf("============================================================\n\n");
    return (int)Unity.TestFailures;
}

void UnityTestFail(uint32_t line, const char *message)
{
    Unity.TestFailures++;
    printf("  [FAIL] Line %u: %s (%s)\n", line, message, Unity.CurrentTestName);
}

void UnityRunTest(void (*testFunc)(void), const char *name, uint32_t line)
{
    Unity.NumberOfTests++;
    Unity.CurrentTestName = name;
    Unity.CurrentTestLineNumber = line;

    setUp();
    uint32_t failures_before = Unity.TestFailures;
    testFunc();
    tearDown();

    if (Unity.TestFailures == failures_before) {
        printf("  [PASS] %-45s (Line %u)\n", name, line);
    }
}
