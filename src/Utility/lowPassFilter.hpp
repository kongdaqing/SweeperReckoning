#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H
#include <Eigen/Dense>
#define M_2PI 3.1415926

template <class T>
class DigitalLPF {
public:
    DigitalLPF();
    // add a new raw value to the filter, retrieve the filtered result
    T apply(const T &sample, double cutoff_freq, double dt);
    T apply(const T &sample);

    void compute_alpha(double sample_freq, double cutoff_freq);

    // get latest filtered value from filter (equal to the value returned by latest call to apply method)
    const T &get() const;
    void reset(T value);

private:
    T _output;
    double alpha = 1.0f;
};

// LPF base class
template <class T>
class LowPassFilter {
public:
    LowPassFilter();
    LowPassFilter(double cutoff_freq);
    LowPassFilter(double sample_freq, double cutoff_freq);

    // change parameters
    void set_cutoff_frequency(double cutoff_freq);
    void set_cutoff_frequency(double sample_freq, double cutoff_freq);

    // return the cutoff frequency
    float get_cutoff_freq(void) const;
    T apply(T sample, double dt);
    T apply(T sample);
    const T &get() const;
    void reset(T value);
    void reset(void) { reset(T()); }

protected:
    double _cutoff_freq;

private:
    DigitalLPF<T> _filter;
};


// typedefs for compatibility
typedef LowPassFilter<int>      LowPassFilterInt;
typedef LowPassFilter<long>     LowPassFilterLong;
typedef LowPassFilter<double>    LowPassFilterDouble;
typedef LowPassFilter<Eigen::Vector2d> LowPassFilterVector2d;
typedef LowPassFilter<Eigen::Vector3d> LowPassFilterVector3d;

#endif
