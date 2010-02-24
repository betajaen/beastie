#ifndef SOY_H
#define SOY_H

#include <string>
#include <iostream>
#include <sstream>
#include "beastie.h"

struct SoyResult
{
 bool        fail;
 int         test_nb;
 std::string test_description;
 std::string case_description;
 std::string fail_message;
};

struct SoyCase
{
 SoyResult R;
};

#define CASES_BEGIN     void Soy() {
#define CASE(TEST_NAME) {SoyResult r = TEST_NAME::Perform();if (r.fail){std::cout << " -- FAILED\n\nCase: #" << r.test_nb << " (" << r.test_description << ")\nReason: " << r.fail_message <<"\n";return;}else{std::cout<<" -- OK\n";}}
#define CASES_END        std::cout << "** OK\n";}

#define CASE_BEGIN(CASE_NAME) struct CASE_NAME : public SoyCase { static SoyResult Perform() { std::cout<<std::string(typeid(CASE_NAME).name()).substr(7);CASE_NAME c; return c.R; }
#define CASE_END };

#define CASE_DESCRIPTION(CASE_DESCRIPTION) R.test_nb = 0; R.case_description = CASE_DESCRIPTION; R.fail = false;
#define TEST_DESCRIPTION(TEST_DESCRIPTION) R.test_nb++;   R.test_description = TEST_DESCRIPTION;

#define TEST_FAIL_IF(COND, FAIL_MESSAGE) if (COND) {R.fail=true;R.fail_message=FAIL_MESSAGE;return;}

std::string uintToString(unsigned int i)
{
 std::stringstream s;
 s << i;
 return s.str();
}

#endif