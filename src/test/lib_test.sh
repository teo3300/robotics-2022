#!/bin/sh

set -e

echo Compiling libraries...
g++ -g -c -o ../constants/constants.{o,cpp} &&\
g++ -g -c -o ../geometry/matrix.{o,cpp} &&\
g++ -g -c -o ../odometry/speed_calculator.{o,cpp} &&\
g++ -g -c -o ../odometry/integrator.{o,cpp}

echo DONE

FLAGS="-I../first_robotics_project_2022/src -L../first_robotics_project_2022/src"
VAL_FLAGS="--error-exitcode=1 --quiet"

echo Running tests...

g++ -g -o ../geometry/matrix_test.{o,cpp}\
          ../geometry/matrix.o &&\
valgrind $VAL_FLAGS ../geometry/matrix_test.o &&\
g++ -g -o ../odometry/speed_calculator_test.{o,cpp}\
          ../odometry/speed_calculator.o\
          ../geometry/matrix.o\
          ../constants/constants.o &&\
valgrind $VAL_FLAGS ../odometry/speed_calculator_test.o 100 100 100 100 &&\
g++ -g -o ../odometry/integrator_test.{o,cpp}\
          ../odometry/integrator.o\
          ../odometry/speed_calculator.o\
          ../geometry/matrix.o\
          ../constants/constants.o &&\
valgrind $VAL_FLAGS ../odometry/integrator_test.o

echo DONE