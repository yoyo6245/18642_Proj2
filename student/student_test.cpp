/*
 * Originally by Phillip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:YUAN-YU LEE
 * ANDREW ID:yuanyul
 * LAST UPDATE:2021/11/05
 *
 */

#include "student_mock.h"
#include <CUnit/Basic.h>
static const int Orientation_num = 4;
static const int turn_back_flag = 3;
using namespace std;

/**
  * state = Initial
  */
void test_t1() {
  setState(Initial);
  setTime_cnt(0);
  bool bumped = true;
  bool atend = false;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  CU_ASSERT_EQUAL(final_move, RIGHT);
  CU_ASSERT_EQUAL(getState(), Detect);
}

/**
  * state = Detect
  * bumped = true
  */

void test_t2() {
  setState(Detect);
  setTurn_cnt(turn_back_flag - 1);
  setTime_cnt(0);
  bool bumped = true;
  bool atend = false;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  
  CU_ASSERT_EQUAL(final_move, NO_MOVE);
  CU_ASSERT_EQUAL(getState(), Bump);
}


/**
  * state = Bump
  */

void test_t3() {
  setState(Bump);
  setTime_cnt(0);
  bool bumped = true;
  bool atend = false;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  
  CU_ASSERT_EQUAL(final_move, LEFT);
  CU_ASSERT_EQUAL(getState(), Detect);
}


/**
  * state = Detect
  * turn_cnt > 3
  */

void test_t4() {
  setState(Detect);
  setTime_cnt(0);
  setTurn_cnt(turn_back_flag + 1);
  set_came_from();
  bool bumped = true;
  bool atend = false;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  CU_ASSERT_EQUAL(final_move, NO_MOVE);
  CU_ASSERT_EQUAL(getState(), Go_back);
}


/**
  * state = Detect
  * turn_cnt < 3
  * bump = false
  */

void test_t5() {
  setState(Detect);
  setTime_cnt(0);
  setTurn_cnt(turn_back_flag - 1);
  bool bumped = false;
  bool atend = false;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  
  CU_ASSERT_EQUAL(final_move, NO_MOVE);
  CU_ASSERT_EQUAL(getState(), Wont_bump);
}


/**
  * state = Wont_bump
  * bump = false
  */

void test_t6() {
  setState(Wont_bump);
  setTime_cnt(0);
  setTurn_cnt(turn_back_flag - 1);
  bool bumped = false;
  bool atend = false;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  
  CU_ASSERT_EQUAL(final_move, MOVE);
  CU_ASSERT_EQUAL(getState(), Initial);
}


/**
  * state = Go_back
  * bump = false
  */

void test_t7() {
  setState(Go_back);
  setTime_cnt(0);
  setTurn_cnt(turn_back_flag + 1);
  bool bumped = false;
  bool atend = false;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  
  CU_ASSERT_EQUAL(final_move, MOVE);
  CU_ASSERT_EQUAL(getState(), Initial);
}


/**
  * state = Initial
  * atend = true
  */

void test_t8() {
  setState(Initial);
  setTime_cnt(0);
  setTurn_cnt(turn_back_flag - 1);
  bool bumped = false;
  bool atend = true;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  
  CU_ASSERT_EQUAL(final_move, NO_MOVE);
  CU_ASSERT_EQUAL(getState(), Initial);
}

/**
  * state = Initial
  * bumped = false
  */
void test_d1() {
  setState(Initial);
  setTime_cnt(0);
  bool bumped = false;
  bool atend = false;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  CU_ASSERT_EQUAL(final_move, RIGHT);
  CU_ASSERT_EQUAL(getState(), Detect);
}

/**
  * state = Detect
  * bumped = true
  * atend = true
  */

void test_d2() {
  setState(Detect);
  setTurn_cnt(turn_back_flag - 1);
  setTime_cnt(0);
  bool bumped = true;
  bool atend = true;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  
  CU_ASSERT_EQUAL(final_move, NO_MOVE);
  CU_ASSERT_EQUAL(getState(), Detect);
}

/**
  * state = Bump
  * atend = true
  */

void test_d3() {
  setState(Bump);
  setTime_cnt(0);
  bool bumped = true;
  bool atend = true;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  
  CU_ASSERT_EQUAL(final_move, NO_MOVE);
  CU_ASSERT_EQUAL(getState(), Bump);
}

/**
  * state = Detect
  * turn_cnt > 3
  * atend = true
  */

void test_d4() {
  setState(Detect);
  setTime_cnt(0);
  setTurn_cnt(turn_back_flag + 1);
  set_came_from();
  bool bumped = true;
  bool atend = true;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  CU_ASSERT_EQUAL(final_move, NO_MOVE);
  CU_ASSERT_EQUAL(getState(), Detect);
}


/**
  * state = Detect
  * turn_cnt < 3
  * bump = false
  * atend = true
  */

void test_d5() {
  setState(Detect);
  setTime_cnt(0);
  setTurn_cnt(turn_back_flag - 1);
  bool bumped = false;
  bool atend = true;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  
  CU_ASSERT_EQUAL(final_move, NO_MOVE);
  CU_ASSERT_EQUAL(getState(), Detect);
}


/**
  * state = Wont_bump
  * bump = false
  */

void test_d6() {
  setState(Wont_bump);
  setTime_cnt(0);
  setTurn_cnt(turn_back_flag - 1);
  bool bumped = false;
  bool atend = true;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  
  CU_ASSERT_EQUAL(final_move, NO_MOVE);
  CU_ASSERT_EQUAL(getState(), Wont_bump);
}


/**
  * state = Go_back
  * bump = false
  * atend = true
  */

void test_d7() {
  setState(Go_back);
  setTime_cnt(0);
  setTurn_cnt(turn_back_flag + 1);
  bool bumped = false;
  bool atend = true;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  
  CU_ASSERT_EQUAL(final_move, NO_MOVE);
  CU_ASSERT_EQUAL(getState(), Go_back);
}

void test_turn_right(){
  // Turn right from Left
  setOrientation(Left);
  turn_right();
  CU_ASSERT_EQUAL(getOrientation(), Up);
  // Turn right from Up
  setOrientation(Up);
  turn_right();
  CU_ASSERT_EQUAL(getOrientation(), Right);
  // Turn right from Right
  setOrientation(Right);
  turn_right();
  CU_ASSERT_EQUAL(getOrientation(), Down);
  // Turn right from Down
  setOrientation(Down);
  turn_right();
  CU_ASSERT_EQUAL(getOrientation(), Left);
}

void test_turn_right_error(){
  setOrientation(Error_orientation);
  setTime_cnt(0);
  setMockerror(false);
  turn_right();
  CU_ASSERT_EQUAL(getMockerror(), true);
}

void test_turn_left(){
  // Turn left from Left
  setOrientation(Left);
  turn_left();
  CU_ASSERT_EQUAL(getOrientation(), Down);
  // Turn left from Up
  setOrientation(Up);
  turn_left();
  CU_ASSERT_EQUAL(getOrientation(), Left);
  // Turn left from Right
  setOrientation(Right);
  turn_left();
  CU_ASSERT_EQUAL(getOrientation(), Up);
  // Turn left from Down
  setOrientation(Down);
  turn_left();
  CU_ASSERT_EQUAL(getOrientation(), Right);
}

void test_turn_left_error(){
  setOrientation(Error_orientation);
  setTime_cnt(0);
  setMockerror(false);
  turn_left();
  CU_ASSERT_EQUAL(getMockerror(), true);
}

void test_update_position(){
  // goto left
  Position origin_pos, update_pos;
  origin_pos = get_position();
  setOrientation(Left);
  new_update_position();
  update_pos = get_position();
  CU_ASSERT_EQUAL(origin_pos.x - 1, update_pos.x);
  // goto up
  origin_pos = get_position();
  setOrientation(Up);
  new_update_position();
  update_pos = get_position();
  CU_ASSERT_EQUAL(origin_pos.y - 1, update_pos.y);
  // goto Right
  origin_pos = get_position();
  setOrientation(Right);
  new_update_position();
  update_pos = get_position();
  CU_ASSERT_EQUAL(origin_pos.x + 1, update_pos.x);
  // goto Down
  origin_pos = get_position();
  setOrientation(Down);
  new_update_position();
  update_pos = get_position();
  CU_ASSERT_EQUAL(origin_pos.y + 1, update_pos.y);
}

void test_update_position_error(){
  Position origin_pos, update_pos;
  origin_pos = get_position();
  setMockerror(false);
  setOrientation(Error_orientation);
  new_update_position();
  update_pos = get_position();
  CU_ASSERT_EQUAL(getMockerror(), true);
}


void test_facing_position(){
  // goto left
  Position origin_pos, facing_pos;
  origin_pos = get_position();
  setOrientation(Left);
  facing_pos = get_facing_pos();
  CU_ASSERT_EQUAL(origin_pos.x - 1, facing_pos.x);
  // goto up
  origin_pos = get_position();
  setOrientation(Up);
  facing_pos = get_facing_pos();
  CU_ASSERT_EQUAL(origin_pos.y - 1, facing_pos.y);
  // goto Right
  origin_pos = get_position();
  setOrientation(Right);
  facing_pos = get_facing_pos();
  CU_ASSERT_EQUAL(origin_pos.x + 1, facing_pos.x);
  // goto Down
  origin_pos = get_position();
  setOrientation(Down);
  facing_pos = get_facing_pos();
  CU_ASSERT_EQUAL(origin_pos.y + 1, facing_pos.y);
}

void test_facing_position_error(){
  Position origin_pos, facing_pos;
  origin_pos = get_position();
  setMockerror(false);
  setOrientation(Error_orientation);
  facing_pos = get_facing_pos();
  CU_ASSERT_EQUAL(getMockerror(), true);
}




void test_error_state(){
  setState(Error_state);
  setTime_cnt(0);
  setMockerror(false);
  bool bumped = true;
  bool atend = false;
  turtleMove final_move = studentTurtleStep(bumped, atend);
  CU_ASSERT_EQUAL(getMockerror(), true);
}




int init() {
  // Any test initialization code goes here
  return 0;
}

int cleanup() {
  // Any test cleanup code goes here
  return 0;
}

/* Skeleton code from http://cunit.sourceforge.net/example.html */
int main() {

  CU_pSuite pSuite1 = NULL;
  CU_pSuite pSuite2 = NULL;
  CU_pSuite pSuite3 = NULL;

  /* initialize the CUnit test registry */
  if (CUE_SUCCESS != CU_initialize_registry())
    return CU_get_error();

  /* add a suite to the registry */
  pSuite1 = CU_add_suite("100% transition coverage", init, cleanup);
  pSuite2 = CU_add_suite("100% data coverage", init, cleanup);
  pSuite3 = CU_add_suite("100% branch coverage", init, cleanup);
  if (NULL == pSuite1 || NULL == pSuite2 || NULL == pSuite3) {
    CU_cleanup_registry();
    return CU_get_error();
  }

  /* add the tests to the suite */
  if ((NULL == CU_add_test(pSuite1, "test of transition T1", test_t1)) ||
      (NULL == CU_add_test(pSuite1, "test of transition T2", test_t2)) ||
      (NULL == CU_add_test(pSuite1, "test of transition T3", test_t3)) ||
      (NULL == CU_add_test(pSuite1, "test of transition T4", test_t4)) ||
      (NULL == CU_add_test(pSuite1, "test of transition T5", test_t5)) ||
      (NULL == CU_add_test(pSuite1, "test of transition T6", test_t6)) ||
      (NULL == CU_add_test(pSuite1, "test of transition T7", test_t7)) ||
      (NULL == CU_add_test(pSuite1, "test of transition T8", test_t8)) ||
      (NULL == CU_add_test(pSuite2, "test of transition D1", test_d1)) ||
      (NULL == CU_add_test(pSuite2, "test of transition D2", test_d2)) ||
      (NULL == CU_add_test(pSuite2, "test of transition D3", test_d3)) ||
      (NULL == CU_add_test(pSuite2, "test of transition D4", test_d4)) ||
      (NULL == CU_add_test(pSuite2, "test of transition D5", test_d5)) ||
      (NULL == CU_add_test(pSuite2, "test of transition D6", test_d6)) ||
      (NULL == CU_add_test(pSuite2, "test of transition D7", test_d7)) ||
      (NULL == CU_add_test(pSuite3, "test of turn right", test_turn_right))||
      (NULL == CU_add_test(pSuite3, "test of turn right", test_turn_left))||
      (NULL == CU_add_test(pSuite3, "test of update position", test_update_position)) ||
      (NULL == CU_add_test(pSuite3, "test of facing position", test_facing_position)) ||
      (NULL == CU_add_test(pSuite3, "test of error state", test_error_state)) ||
      (NULL == CU_add_test(pSuite3, "test of error orientation in turn right function", test_turn_right_error)) ||
      (NULL == CU_add_test(pSuite3, "test of error orientation in turn left function", test_turn_left_error)) || 
      (NULL == CU_add_test(pSuite3, "test of error orientation in update_position function", test_update_position_error)) ||
      (NULL == CU_add_test(pSuite3, "test of error orientation in facing_position function", test_facing_position_error))


      )
    {
      CU_cleanup_registry();
      return CU_get_error();
    }
  
  /* Run all tests using the CUnit Basic interface */
  CU_basic_set_mode(CU_BRM_VERBOSE);
  CU_basic_run_tests();
  CU_cleanup_registry();
  return CU_get_error();

  return 0;
}


