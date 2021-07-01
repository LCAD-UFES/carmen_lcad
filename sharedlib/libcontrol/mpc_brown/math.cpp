#include <cstdio>
#include <cmath>
#include <vector>
#include <string>
#include <map>

using namespace std;

double
cumtrapz(vector <double> y,  vector <double> x, double x0)
{
    double h = (x[0] - x[x.size() - 1]) / x.size();
    double i = x0 + h;
    double soma = 0.0;
    for (int j = 0; j < y.size() - 2; j++)
    {
        soma = soma + y[j];
        i = i + x[j + 1];
    }
    double r = h * ((y[0] + y[y.size() - 1]) / 2 + soma);
    return r;
}

vector <double> 
clamp (vector<double> x, double lo , double hi)
{
    vector<double> lambda;
    for(int i = 0; i < x.size(); i++)
    {
        if(x[i] < lo) 
        {
            lambda.push_back(lo); //return lo;
        }else
        {
            if(x[i] > hi)
            {
                lambda.push_back(hi);
            }else
            {
                lambda.push_back(x[i]);
            }
        }
    }
    return lambda;
}

vector <double>
distance2(vector <double> a, vector <double> b,vector <double>  x)
{
    vector <double> v;
    for(int i = 0; i < a.size(); i++)
    {
        v.push_back(b[i] - a[i]);
    }
    vector <double> c;
    for(int i = 0; i < v.size(); i++)
    {
        c.push_back(v[i] * (x[i] - a[i]) / (v[i] * v[i]));
    }
    vector<double> lambda = clamp(c, 0, 1);
    vector<double> p;
    for(int i = 0; i < lambda.size(); i++)
    {
        p.push_back((1 - lambda[i]) * a[i] + lambda[i] * b[i]);
    }
    vector<double> aux;
    for(int i = 0; i < p.size(); i++)
    {
        aux.push_back((p[i] - x[i]) * (p[i] - x[i]));
    }
    return aux;
}
