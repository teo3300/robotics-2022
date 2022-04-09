#!/bin/sh

set -e

echo Compiling libraries...
g++ -g -c -o ../constants/constants.{o,cpp} &&\
g++ -g -c -o ../geometry/matrix.{o,cpp} &&\
g++ -g -c -o ../odometry/odometry.{o,cpp} &&\
g++ -g -c -o ../integration/integrator.{o,cpp}

echo DONE

VAL_FLAGS="--error-exitcode=1 --quiet"

echo Running tests...

g++ -g -o ../geometry/matrix_test.{o,cpp}\
          ../geometry/matrix.o &&\
valgrind $VAL_FLAGS ../geometry/matrix_test.o &&\
g++ -g -o ../odometry/odometry_test.{o,cpp}\
          ../odometry/odometry.o\
          ../geometry/matrix.o\
          ../constants/constants.o &&\
valgrind $VAL_FLAGS ../odometry/odometry_test.o 100 100 100 100 &&\
g++ -g -o ../integration/integrator_test.{o,cpp}\
          ../integration/integrator.o\
          ../odometry/odometry.o\
          ../geometry/matrix.o\
          ../constants/constants.o &&\
valgrind $VAL_FLAGS ../integration/integrator_test.o

echo DONE