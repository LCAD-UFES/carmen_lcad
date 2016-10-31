#ifndef ASTAR_VECTOR_2D_ARRAY_HPP
#define ASTAR_VECTOR_2D_ARRAY_HPP

#include <vector>
#include <exception>

#include "Vector2D.hpp"

namespace smoother {

template<typename T>
class Vector2DArray {

    public:

        // the current array
        std::vector<smoother::Vector2D<T>> vs;

        // compare two Vector2DArray
        bool EqualsTo(const smoother::Vector2DArray<T> &b) {

            if (vs.size() == b.vs.size()) {

                // get the current size
                unsigned int v_size = vs.size();

                // get the direct access
                const std::vector<smoother::Vector2D<T>> &vectors(b.vs);

                for (unsigned int i = 0; i < v_size; ++i) {

                    if (vectors[i] != vs[i]) {

                        return false;

                    }

                }

            }

            return false;

        }
        // == operator overloading

        // reset all values in a given position array
        static void SetZero(std::vector<smoother::Vector2D<T>> &v) {

            // get the vector size
            unsigned int v_size = v.size();

            for (unsigned int i = 0; i < v_size; ++i) {

                v[i].x = 0.0;
                v[i].y = 0.0;

            }

        }

        // custom dot product
        static T DotProduct(
                const std::vector<smoother::Vector2D<T>> &a, const std::vector<smoother::Vector2D<T>> &b)
        {
            // the resulting dot product
            T dot = (T) 0.0;

            if (a.size() == b.size()) {

                // the vector size
                unsigned int size = a.size();

                for (unsigned int i = 0; i < size; ++i) {

                    dot += a[i].x * b[i].x + a[i].y * b[i].y;

                }

                return dot;

            } else {

                // exception
                throw std::exception();

            }

        }

        // scale a given Vector2D array
        static void ScaleVector(T value, std::vector<smoother::Vector2D<T>> &v) {

            // get the vector size
            unsigned int v_size = v.size();

            for (unsigned int i = 0; i < v_size; ++i) {

                v[i].x *= value;
                v[i].y *= value;

            }

        }

        // add the first vector2D array to the second one
        static void AddVectors(const std::vector<smoother::Vector2D<T>> &a, std::vector<smoother::Vector2D<T>> &b) {

            // get the size
            unsigned int size = a.size();

            if (size == b.size()) {

                for (unsigned int i = 0; i < size; ++i) {

                    b[i].x += a[i].x;
                    b[i].y += a[i].y;

                }

            }

        }

        // subtract two vectors and store the result at the third one
        // C == A - B
        static void SubtractCAB(std::vector<smoother::Vector2D<T>> &c, const std::vector<smoother::Vector2D<T>> &a, const std::vector<smoother::Vector2D<T>> &b) {

            // get the size
            unsigned int size = a.size();

            if (b.size() == size) {

                if (c.size() != size) {

                    c = a;

                    for (unsigned int i = 0; i < size; ++i) {

                        c[i].x -= b[i].x;
                        c[i].y -= b[i].y;

                    }

                    return;

                }

                // subtract A - B and store in C
                for (unsigned int i = 0; i < size; ++i) {

                    c[i].x = a[i].x - b[i].x;
                    c[i].y = a[i].y - b[i].y;

                }

                return;

            }

            // throwh exception
            throw std::exception();

        }

        // compute a norm 2 for a Vector2D array
        static T VectorNorm2(const std::vector<smoother::Vector2D<T>> &v) {

            // the resulting norm
            T norm = (T) 0.0;

            // get the size
            unsigned int size = v.size();

            for (unsigned int i = 0; i < size; ++i) {

                norm += v[i].Norm2();

            }

            return norm;

        }

        // verify if 2 vectors have the same values
        static bool EqualVectors(const std::vector<smoother::State2D> &current, const std::vector<smoother::State2D> &next) {

            // the resulting boolean
            bool equal = true;

            // get the current vector size
            unsigned int c_size = current.size();

            if (c_size == next.size()) {

                // iterate over the two vectors and compare each element
                for (unsigned int i = 0; i < c_size; ++i) {

                    if (current[i].position != next[i].position) {

                        equal = false;

                    }

                }

            }

            return equal;

        }


};

template<typename T>
using Vector2DArrayPtr = Vector2DArray<double>*;

}

#endif
