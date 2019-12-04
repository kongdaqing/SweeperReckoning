#include "lowPassFilter.hpp"
#include "utility.hpp"


template <class T>
DigitalLPF<T>::DigitalLPF() {
  // built in initialization
  _output = T();
}

// add a new raw value to the filter, retrieve the filtered result
template <class T>
T DigitalLPF<T>::apply(const T &sample, double cutoff_freq, double dt) {
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        _output = sample;
        return _output;
    }
    double rc = 1.0f/(M_2PI*cutoff_freq);
    alpha = Utility::ConstriantDouble(dt/(dt+rc), 0.0, 1.0);
    _output += (sample - _output) * alpha;
    return _output;
}

template <class T>
T DigitalLPF<T>::apply(const T &sample) {
    _output += (sample - _output) * alpha;
    return _output;
}

template <class T>
void DigitalLPF<T>::compute_alpha(double sample_freq, double cutoff_freq) {
    if (cutoff_freq <= 0.0f || sample_freq <= 0.0f) {
        alpha = 1.0;
    } else {
        double dt = 1.0/sample_freq;
        double rc = 1.0f/(M_2PI*cutoff_freq);
        alpha = Utility::ConstriantDouble(dt/(dt+rc), 0.0f, 1.0f);
    }
}

// get latest filtered value from filter (equal to the value returned by latest call to apply method)
template <class T>
const T &DigitalLPF<T>::get() const {
    return _output;
}

template <class T>
void DigitalLPF<T>::reset(T value) {
    _output = value;
}

////////////////////////////////////////////////////////////////////////////////////////////
// LowPassFilter
////////////////////////////////////////////////////////////////////////////////////////////

// constructors
template <class T>
LowPassFilter<T>::LowPassFilter() :
    _cutoff_freq(0.0f) {}

template <class T>
LowPassFilter<T>::LowPassFilter(double cutoff_freq) :
    _cutoff_freq(cutoff_freq) {}

template <class T>
LowPassFilter<T>::LowPassFilter(double sample_freq, double cutoff_freq)
{
    set_cutoff_frequency(sample_freq, cutoff_freq);
}

// change parameters
template <class T>
void LowPassFilter<T>::set_cutoff_frequency(double cutoff_freq) {
    _cutoff_freq = cutoff_freq;
}

template <class T>
void LowPassFilter<T>::set_cutoff_frequency(double sample_freq, double cutoff_freq) {
    _cutoff_freq = cutoff_freq;
    _filter.compute_alpha(sample_freq, cutoff_freq);
}

// return the cutoff frequency
template <class T>
float LowPassFilter<T>::get_cutoff_freq(void) const {
    return _cutoff_freq;
}

template <class T>
T LowPassFilter<T>::apply(T sample, double dt) {
    return _filter.apply(sample, _cutoff_freq, dt);
}

template <class T>
T LowPassFilter<T>::apply(T sample) {
    return _filter.apply(sample);
}

template <class T>
const T &LowPassFilter<T>::get() const {
    return _filter.get();
}

template <class T>
void LowPassFilter<T>::reset(T value) {
    _filter.reset(value);
}

/*
 * Make an instances
 * Otherwise we have to move the constructor implementations to the header file :P
 */
template class LowPassFilter<int>;
template class LowPassFilter<long>;
template class LowPassFilter<double>;
template class LowPassFilter<Eigen::Vector2d>;
template class LowPassFilter<Eigen::Vector3d>;

