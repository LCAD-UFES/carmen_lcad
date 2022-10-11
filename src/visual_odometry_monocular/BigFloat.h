/*
 * BigFloat - Arbitrary-Precision Float Arithmetic Library
 * Copyright (C) 2015 Gianfranco Mariotti
*/

#ifndef BIGFLOAT_H
#define BIGFLOAT_H

#include <iostream>
#include <deque>
#include <cmath>
#include <sstream>

class BigFloat {
private:
    char sign;
    std::deque<char> number;
    int decimals;

    //Error Checking
    bool error;
    std::string error_code;

    //Transformations int<-->char single digit
    inline static int CharToInt(const char& val)
    {
        return (val - '0');
    };
    inline static char IntToChar(const int& val)
    {
        return (val + '0');
    };

    //Comparator without sign, utilized by Comparators and Operations
    static int CompareNum(const BigFloat& left, const BigFloat& right);

    //Operations without sign and decimals, utilized by Operations
    static BigFloat Sum(const BigFloat& left, const BigFloat& right);
    static BigFloat Subtract(const BigFloat& left, const BigFloat& right);
    static BigFloat Multiply(const BigFloat& left, const BigFloat& right);
    static BigFloat Pow(const BigFloat& left, const BigFloat& right);

public:
    //Constructors
    BigFloat()
    {
        sign = '\0';
        decimals = 0;
        error = 1;
        error_code = "Unset";
    };
    BigFloat(const char* strNum)
    {
        *this = strNum;
    };
    BigFloat(std::string strNum)
    {
        *this = strNum;
    };
    BigFloat(int Num)
    {
        *this = Num;
    };
    BigFloat(double Num)
    {
        *this = Num;
    };

    //Assignment operators
    BigFloat& operator=(const char* strNum);
    BigFloat& operator=(std::string strNum);
    BigFloat& operator=(int Num);
    BigFloat& operator=(double Num);

    //Operations
    friend BigFloat operator+(const BigFloat& left_, const BigFloat& right_);
    friend BigFloat operator+(const BigFloat& left, const int& int_right);
    friend BigFloat operator+(const BigFloat& left, const double& double_right);

    friend BigFloat operator-(const BigFloat& left_, const BigFloat& right_);
    friend BigFloat operator-(const BigFloat& left, const int& int_right);
    friend BigFloat operator-(const BigFloat& left, const double& double_right);

    friend BigFloat operator*(const BigFloat& left, const BigFloat& right);
    friend BigFloat operator*(const BigFloat& left, const int& int_right);
    friend BigFloat operator*(const BigFloat& left, const double& double_right);

    friend BigFloat operator/(const BigFloat& left, const BigFloat& right);
    friend BigFloat operator/(const BigFloat& left, const int& int_right);
    friend BigFloat operator/(const BigFloat& left, const double& double_right);
    static BigFloat PrecDiv(const BigFloat& left, const BigFloat& right, int div_precision);
    static BigFloat PrecDiv(const BigFloat& left, const int& int_right, int div_precision);
    static BigFloat PrecDiv(const BigFloat& left, const double& double_right, int div_precision);

    friend BigFloat operator%(const BigFloat& left, const BigFloat& right);
    friend BigFloat operator%(const BigFloat& left, const int& int_right);

    static BigFloat Power(const BigFloat& left, const BigFloat& right, int div_precision = 0);
    static BigFloat Power(const BigFloat& left, const int& int_right, int div_precision = 0);
    static BigFloat Power(const BigFloat& left, const double& double_right, int div_precision = 0);

    BigFloat& operator++(int i)
    {
        *this = *this + 1;
        return *this;
    };
    BigFloat& operator++()
    {
        *this = *this + 1;
        return *this;
    };
    BigFloat& operator--(int i)
    {
        *this = *this - 1;
        return *this;
    };
    BigFloat& operator--()
    {
        *this = *this - 1;
        return *this;
    };

    //Comparators
    bool operator== (const BigFloat& right) const;
    bool operator== (const int& int_right) const;
    bool operator== (const double& double_right) const;

    bool operator!= (const BigFloat& right) const;
    bool operator!= (const int& int_right) const;
    bool operator!= (const double& double_right) const;

    bool operator> (const BigFloat& right) const;
    bool operator> (const int& int_right) const;
    bool operator> (const double& double_right) const;

    bool operator>= (const BigFloat& right) const;
    bool operator>= (const int& int_right) const;
    bool operator>= (const double& double_right) const;

    bool operator< (const BigFloat& right) const;
    bool operator< (const int& int_right) const;
    bool operator< (const double& double_right) const;

    bool operator<= (const BigFloat& right) const;
    bool operator<= (const int& int_right) const;
    bool operator<= (const double& double_right) const;

    //Stream Operators
    friend std::ostream& operator<<(std::ostream& out, const BigFloat& right);
    friend std::istream& operator>>(std::istream& in, BigFloat& right);

    //Transformation Methods
    double ToDouble() const;
    float ToFloat() const;
    std::string ToString() const;
    void SetPrecision(int prec); //Approximate number or Increase number decimals

    void LeadTrim(); //Remove number leading zeros, utilized by Operations without sign
    void TrailTrim(); //Remove number non significant trailing zeros

    //Error Checking
    inline bool HasError() const
    {
        return error;
    };
    inline std::string GetError() const
    {
        if(error)
            return error_code;
        else
            return "No Error";
    };

    //Miscellaneous Methods
    inline int Decimals() const
    {
        return decimals;
    };
    inline int Ints() const
    {
        return number.size() - decimals;
    };
    inline int MemorySize() const
    {
        return sizeof(*this) + number.size() * sizeof(char);
    };
    std::string Exp() const;

};

#endif
