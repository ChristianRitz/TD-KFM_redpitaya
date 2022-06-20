#ifndef OPTIMIZEDKELVINESTIMATOREKF_H
#define OPTIMIZEDKELVINESTIMATOREKF_H
#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include<cmath>

#define PI 3.1415926535897

template <typename scalar_t, int filter_order=2>
class OptimizedKelvinEstimatorEKF {

public:
    typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> matrix_t;

    // State vectors
    typedef Eigen::Matrix<scalar_t, filter_order+3, 1> state_vector_t;

    // Covariance matrix
    typedef Eigen::Matrix<scalar_t, filter_order+3, filter_order+3> state_covariance_t;

    // Filter coefficients
    typedef Eigen::Matrix<scalar_t, filter_order+1, 1> filter_coefficient_t;

    // Combined system matrices
    typedef Eigen::Matrix<scalar_t, filter_order+3, filter_order+3> transition_matrix_t;
    typedef Eigen::Matrix<scalar_t, 1, filter_order+3> output_matrix_t;


    OptimizedKelvinEstimatorEKF() {
        reset_filter();
        setup();
    }

    void setup() {
        // Initialize system states
        state_estimate.setZero();
        state_covariance.setIdentity();

        // Initialize covariances
        transition_covariance.setZero();
        set_system_noise(1, 1, 1);
        set_frequency_noise(1);
        set_measurement_noise(1);

        // Initialize df_error
        df_error = 0;

        // Reset fading memory alpha
        set_fading_memory_alpha(1.0);

        last_voltage = 0;
    }

    void reset_estimation(){
        state_estimate.setZero();
        state_covariance.setIdentity()*1000;
    }


    void reset_filter() {
        // Set filter to feedthrough only.
        filter_coefficients_numer.setZero();
        filter_coefficients_numer(0) = 1;
        filter_coefficients_denom.setZero();
        filter_coefficients_denom(0) = 1;
    }
    
    long int bin (long int n, long int k){

        long int i;

        long int binomialkoeffizient = 1;

        if (n < 0 || n > 20 || k < 0 || k > n) return (-1);

        if (k == 0 || k == n) return (binomialkoeffizient);

        for (i = 1; i <= k; i++){

        binomialkoeffizient = binomialkoeffizient * (n - i + 1) / i;

        }
        return (binomialkoeffizient);

    }


    void setup_filter(double pll_bw, double sample_rate, int order=2) {

    // Create filter matrices from filter coefficients b and a.
    // NOTE: Implementation requires a(0) == 1.

        filter_coefficient_t pll_a, pll_b;

        double warped_bandwidth = 2* sample_rate * std::atan(pll_bw / (2 * sample_rate));
        double tau = sqrt(pow(2,(1/(double)order))-1) / (2*PI*warped_bandwidth);
        double c = 2 * tau * sample_rate;

        for(int i=0; i<filter_order+1;i++) {

            pll_b(i) = bin(filter_order, i);
            
            pll_a(i) = bin(filter_order, i) * std::pow((1 + c),(filter_order-i)) * std::pow((1 - c),i);
            
        }

        filter_coefficients_numer = pll_b/pll_a(0);

        filter_coefficients_denom = pll_a/pll_a(0);

    }


    void set_measurement_noise(const scalar_t R) {
        measurement_covariance = R;
    }

    void set_frequency_noise(const scalar_t R) {
        transition_covariance(3, 3) = R;
    }

    void set_system_noise(const scalar_t Q_dfstat, const scalar_t Q_cpd, const scalar_t Q_coeffa) {
        transition_covariance(0, 0) = Q_dfstat;
        transition_covariance(1, 1) = Q_cpd;
        transition_covariance(2, 2) = Q_coeffa;
    }

    void set_fading_memory_alpha(const scalar_t alpha) {
        fading_memory_alphasq = alpha * alpha;
    }

    void update(const scalar_t df, const scalar_t voltage) {

        state_vector_t last_state_estimate = state_estimate;
        state_covariance_t last_state_covariance = state_covariance;

        // 8< -- start generated code -- 8<
const scalar_t x0 = -filter_coefficients_denom(2)*filter_coefficients_numer(0) + filter_coefficients_numer(2);
const scalar_t x1 = -last_state_estimate(1);
const scalar_t x2 = voltage + x1;
const scalar_t x3 = (x2*x2);
const scalar_t x4 = -filter_coefficients_denom(1)*filter_coefficients_numer(0) + filter_coefficients_numer(1);
const scalar_t x5 = last_voltage + x1;
const scalar_t x6 = (x5*x5);
const scalar_t x7 = -filter_coefficients_denom(1)*last_state_estimate(3) - filter_coefficients_denom(2)*last_state_estimate(4) + last_state_estimate(0) - last_state_estimate(2)*x6;
const scalar_t x8 = df - filter_coefficients_numer(0)*(last_state_estimate(0) - last_state_estimate(2)*x3) - last_state_estimate(3)*x0 - x4*x7;
const scalar_t x9 = fading_memory_alphasq;
const scalar_t x10 = last_state_covariance(0, 0)*x9;
const scalar_t x11 = transition_covariance(0, 0) + x10;
const scalar_t x12 = filter_coefficients_numer(0)*x11;
const scalar_t x13 = last_state_covariance(0, 3)*x9;
const scalar_t x14 = x0*x13;
const scalar_t x15 = last_state_covariance(0, 1)*x9;
const scalar_t x16 = filter_coefficients_numer(0)*x15;
const scalar_t x17 = 2*last_state_estimate(2)*x16*x2;
const scalar_t x18 = last_state_covariance(0, 2)*x9;
const scalar_t x19 = filter_coefficients_numer(0)*x18;
const scalar_t x20 = x19*x3;
const scalar_t x21 = last_state_covariance(0, 4)*x9;
const scalar_t x22 = 2*last_state_estimate(2)*x5;
const scalar_t x23 = -filter_coefficients_denom(1)*x13 - filter_coefficients_denom(2)*x21 + x10 + x15*x22 - x18*x6;
const scalar_t x24 = x23*x4;
const scalar_t x25 = x12 + x14 + x17 - x20 + x24;
const scalar_t x26 = filter_coefficients_numer(0)*x25;
const scalar_t x27 = filter_coefficients_numer(0)*x13;
const scalar_t x28 = x0*x9;
const scalar_t x29 = last_state_covariance(3, 3)*x28;
const scalar_t x30 = 2*filter_coefficients_numer(0)*last_state_estimate(2)*x2*x9;
const scalar_t x31 = last_state_covariance(1, 3)*x30;
const scalar_t x32 = filter_coefficients_numer(0)*x3*x9;
const scalar_t x33 = last_state_covariance(2, 3)*x32;
const scalar_t x34 = filter_coefficients_denom(1)*x9;
const scalar_t x35 = filter_coefficients_denom(2)*x9;
const scalar_t x36 = 2*last_state_estimate(2)*x5*x9;
const scalar_t x37 = x6*x9;
const scalar_t x38 = last_state_covariance(1, 3)*x36 - last_state_covariance(2, 3)*x37 - last_state_covariance(3, 3)*x34 - last_state_covariance(3, 4)*x35 + x13;
const scalar_t x39 = x38*x4;
const scalar_t x40 = x27 + x29 + x31 - x33 + x39;
const scalar_t x41 = x0*x40;
const scalar_t x42 = 2*filter_coefficients_numer(0)*last_state_estimate(2)*x2;
const scalar_t x43 = last_state_covariance(1, 3)*x28;
const scalar_t x44 = last_state_covariance(1, 2)*x32;
const scalar_t x45 = last_state_covariance(1, 1)*x9;
const scalar_t x46 = transition_covariance(1, 1) + x45;
const scalar_t x47 = x42*x46;
const scalar_t x48 = -last_state_covariance(1, 2)*x37 - last_state_covariance(1, 3)*x34 - last_state_covariance(1, 4)*x35 + x15 + x22*x45;
const scalar_t x49 = x4*x48;
const scalar_t x50 = x16 + x43 - x44 + x47 + x49;
const scalar_t x51 = x42*x50;
const scalar_t x52 = filter_coefficients_numer(0)*x3;
const scalar_t x53 = last_state_covariance(2, 3)*x28;
const scalar_t x54 = last_state_covariance(1, 2)*x30;
const scalar_t x55 = last_state_covariance(2, 2)*x9;
const scalar_t x56 = transition_covariance(2, 2) + x55;
const scalar_t x57 = x52*x56;
const scalar_t x58 = last_state_covariance(1, 2)*x36 - last_state_covariance(2, 3)*x34 - last_state_covariance(2, 4)*x35 + x18 - x55*x6;
const scalar_t x59 = x4*x58;
const scalar_t x60 = x19 + x53 + x54 - x57 + x59;
const scalar_t x61 = x52*x60;
const scalar_t x62 = filter_coefficients_numer(0)*x23;
const scalar_t x63 = x0*x38;
const scalar_t x64 = x42*x48;
const scalar_t x65 = x52*x58;
const scalar_t x66 = -filter_coefficients_denom(1)*x38 - filter_coefficients_denom(2)*(last_state_covariance(1, 4)*x36 - last_state_covariance(2, 4)*x37 - last_state_covariance(3, 4)*x34 - last_state_covariance(4, 4)*x35 + x21) + x22*x48 + x23 - x58*x6;
const scalar_t x67 = x4*x66;
const scalar_t x68 = x62 + x63 + x64 - x65 + x67;
const scalar_t x69 = x4*x68;
const scalar_t x70 = measurement_covariance + x26 + x41 + x51 - x61 + x69;
const scalar_t x71 = 1.0/x70;
const scalar_t x72 = x25*x71;
const scalar_t x73 = last_state_estimate(0) + x72*x8;
const scalar_t x74 = x50*x71;
const scalar_t x75 = x74*x8;
const scalar_t x76 = x60*x71;
const scalar_t x77 = last_state_estimate(2) + x76*x8;
const scalar_t x78 = x68*x71;
const scalar_t x79 = x7 + x78*x8;
const scalar_t x80 = x40*x71;
const scalar_t x81 = last_state_estimate(3) + x8*x80;
const scalar_t x82 = 1.0/(x70*x70);
const scalar_t x83 = measurement_covariance*x82;
const scalar_t x84 = -x26*x71 + 1;
const scalar_t x85 = x13*x84 - x29*x72 - x31*x72 + x33*x72 - x39*x72;
const scalar_t x86 = x0*x85;
const scalar_t x87 = x11*x84 - x14*x72 - x17*x72 + x20*x72 - x24*x72;
const scalar_t x88 = x18*x84 - x53*x72 - x54*x72 + x57*x72 - x59*x72;
const scalar_t x89 = filter_coefficients_numer(0)*x3*x88;
const scalar_t x90 = x15*x84 - x43*x72 + x44*x72 - x47*x72 - x49*x72;
const scalar_t x91 = 2*filter_coefficients_numer(0)*last_state_estimate(2)*x2*x90;
const scalar_t x92 = x23*x84 - x63*x72 - x64*x72 + x65*x72 - x67*x72;
const scalar_t x93 = x4*x92;
const scalar_t x94 = measurement_covariance*x50*x82;
const scalar_t x95 = filter_coefficients_numer(0)*x50*x71;
const scalar_t x96 = x0*x50*x71;
const scalar_t x97 = filter_coefficients_numer(0)*x3*x50*x71;
const scalar_t x98 = -x51*x71 + 1;
const scalar_t x99 = x4*x50*x71;
const scalar_t x100 = measurement_covariance*x25*x82;
const scalar_t x101 = filter_coefficients_numer(0)*x60*x71;
const scalar_t x102 = x61*x71 + 1;
const scalar_t x103 = filter_coefficients_numer(0)*x68*x71;
const scalar_t x104 = -x69*x71 + 1;
const scalar_t x105 = measurement_covariance*x40*x82;
const scalar_t x106 = filter_coefficients_numer(0)*x40*x71;
const scalar_t x107 = -x41*x71 + 1;
const scalar_t x108 = filter_coefficients_numer(0)*x3*x40*x71;
const scalar_t x109 = 2*filter_coefficients_numer(0)*last_state_estimate(2)*x2*x40*x71;
const scalar_t x110 = x4*x40*x71;
const scalar_t x111 = -x12*x74 - x14*x74 + x15*x98 + x20*x74 - x24*x74;
const scalar_t x112 = last_state_covariance(1, 3)*x9*x98 - x27*x74 - x29*x74 + x33*x74 - x39*x74;
const scalar_t x113 = last_state_covariance(1, 2)*x9;
const scalar_t x114 = x113*x98 - x19*x74 - x53*x74 + x57*x74 - x59*x74;
const scalar_t x115 = -x16*x74 - x43*x74 + x44*x74 + x46*x98 - x49*x74;
const scalar_t x116 = x48*x98 - x62*x74 - x63*x74 + x65*x74 - x67*x74;
const scalar_t x117 = x0*x60*x71;
const scalar_t x118 = 2*filter_coefficients_numer(0)*last_state_estimate(2)*x2*x60*x71;
const scalar_t x119 = x4*x60*x71;
const scalar_t x120 = x0*x68*x71;
const scalar_t x121 = filter_coefficients_numer(0)*x3*x68*x71;
const scalar_t x122 = 2*filter_coefficients_numer(0)*last_state_estimate(2)*x2*x68*x71;
const scalar_t x123 = x102*x18 - x12*x76 - x14*x76 - x17*x76 - x24*x76;
const scalar_t x124 = last_state_covariance(2, 3)*x102*x9 - x27*x76 - x29*x76 - x31*x76 - x39*x76;
const scalar_t x125 = x102*x113 - x16*x76 - x43*x76 - x47*x76 - x49*x76;
const scalar_t x126 = x102*x56 - x19*x76 - x53*x76 - x54*x76 - x59*x76;
const scalar_t x127 = x102*x58 - x62*x76 - x63*x76 - x64*x76 - x67*x76;
const scalar_t x128 = x104*x23 - x12*x78 - x14*x78 - x17*x78 + x20*x78;
const scalar_t x129 = x104*x38 - x27*x78 - x29*x78 - x31*x78 + x33*x78;
const scalar_t x130 = x104*x58 - x19*x78 - x53*x78 - x54*x78 + x57*x78;
const scalar_t x131 = x104*x48 - x16*x78 - x43*x78 + x44*x78 - x47*x78;
const scalar_t x132 = x104*x66 - x62*x78 - x63*x78 - x64*x78 + x65*x78;
const scalar_t x133 = x107*x9;
df_error = df - filter_coefficients_numer(0)*(x73 - x77*((x2 - x75)*(x2 - x75))) - x0*x81 - x4*x79;
kalman_gain(0) = x72;
kalman_gain(1) = x74;
kalman_gain(2) = x76;
kalman_gain(3) = x78;
kalman_gain(4) = x80;
state_covariance(0, 0) = (x25*x25)*x83 - x72*x86 + x72*x89 - x72*x91 - x72*x93 + x84*x87;
state_covariance(0, 1) = x25*x94 - x85*x96 - x87*x95 + x88*x97 + x90*x98 - x92*x99;
state_covariance(0, 2) = x100*x60 - x101*x87 + x102*x88 - x76*x86 - x76*x91 - x76*x93;
state_covariance(0, 3) = x100*x68 - x103*x87 + x104*x92 - x78*x86 + x78*x89 - x78*x91;
state_covariance(0, 4) = x105*x25 - x106*x87 + x107*x85 + x108*x88 - x109*x90 - x110*x92;
state_covariance(1, 1) = -x111*x95 - x112*x96 + x114*x97 + x115*x98 - x116*x99 + (x50*x50)*x83;
state_covariance(1, 2) = -x101*x111 + x102*x114 - x112*x117 - x115*x118 - x116*x119 + x60*x94;
state_covariance(1, 3) = -x103*x111 + x104*x116 - x112*x120 + x114*x121 - x115*x122 + x68*x94;
state_covariance(1, 4) = x105*x50 - x106*x111 + x107*x112 + x108*x114 - x109*x115 - x110*x116;
state_covariance(2, 2) = -x101*x123 + x102*x126 - x117*x124 - x118*x125 - x119*x127 + (x60*x60)*x83;
state_covariance(2, 3) = -x103*x123 + x104*x127 - x120*x124 + x121*x126 - x122*x125 + x60*x68*x83;
state_covariance(2, 4) = x105*x60 - x106*x123 + x107*x124 + x108*x126 - x109*x125 - x110*x127;
state_covariance(3, 3) = -x103*x128 + x104*x132 - x120*x129 + x121*x130 - x122*x131 + (x68*x68)*x83;
state_covariance(3, 4) = x105*x68 - x106*x128 + x107*x129 + x108*x130 - x109*x131 - x110*x132;
state_covariance(4, 4) = -x106*(x107*x13 - x12*x80 - x17*x80 + x20*x80 - x24*x80) + x107*(last_state_covariance(3, 3)*x133 - x27*x80 - x31*x80 + x33*x80 - x39*x80) + x108*(last_state_covariance(2, 3)*x133 - x19*x80 - x54*x80 + x57*x80 - x59*x80) - x109*(last_state_covariance(1, 3)*x133 - x16*x80 + x44*x80 - x47*x80 - x49*x80) - x110*(x107*x38 - x62*x80 - x64*x80 + x65*x80 - x67*x80) + (x40*x40)*x83;
state_estimate(0) = x73;
state_estimate(1) = last_state_estimate(1) + x75;
state_estimate(2) = x77;
state_estimate(3) = x79;
state_estimate(4) = x81;
        // 8< -- stop generated code -- 8<

        // There can be multiple solutions for the quadratic characteristics.
        // When coefficient a < 0 is predicted, correct the state by projecting
        // the estimate to the allowed region. See eq. (25) in [1].
        // [1]: Simon, D. IET Control Theory & Applications 4, 1303–1318 (2010)
        const size_t coeffa_ind = 2;
        if (state_estimate(coeffa_ind) < 0) {
            state_vector_t cov;
            for (size_t i = 0, num_rows = cov.rows(); i < num_rows; ++i) {
                cov(i) = (
                    (i > coeffa_ind)
                    ? state_covariance(coeffa_ind, i)
                    : state_covariance(i, coeffa_ind)
                );
            }
            state_estimate -= (
                cov / cov(coeffa_ind) * state_estimate(coeffa_ind)
            );
            state_estimate(coeffa_ind) = 1e-9;
        }

        last_voltage = voltage;
    }

    matrix_t update_all(
        const Eigen::VectorXd& input_sample_vector, const Eigen::VectorXd& voltage_vector,
        const Eigen::VectorXd& Q_dfstat_vector, const Eigen::VectorXd& Q_cpd_vector,
        const Eigen::VectorXd& Q_coeffa_vector
    ) {
        int n = input_sample_vector.size();
        Eigen::Matrix<scalar_t, 4 + 6, Eigen::Dynamic> data;
        data.resize(4 + 6, n);

        for(int i = 0; i < n; i++){
            set_system_noise(Q_dfstat_vector(i), Q_cpd_vector(i), Q_coeffa_vector(i));
            update(input_sample_vector(i), voltage_vector(i));

            data.col(i).head(3) = get_state_estimate();
            data(3, i) = get_df_error();

            // Output covariance matrix as coefficients along main diagonals
            const state_covariance_t& cov = get_state_covariance();
            int ind = 0;
            for (int diag = 0; diag < 3; ++diag) {
                for (int h = 0; h < 3 - diag; ++h) {
                    data(4 + ind, i) = cov(h, diag + h);
                    ++ind;
                }
            }
        };
        return data;
    }

    const state_vector_t& get_state_estimate() {
        return state_estimate;
    }

    const state_covariance_t& get_state_covariance() {
        return state_covariance;
    }

    scalar_t get_df_stat() {
        return state_estimate(0);
    }

    scalar_t get_cpd() {
        return state_estimate(1);
    }

    scalar_t get_coeff_a() {
        return state_estimate(2);
    }

    scalar_t get_df_error() {
        return df_error;
    }

protected:
    // System state
    state_vector_t state_estimate;
    state_covariance_t state_covariance;
    scalar_t last_voltage;

    // Filter matrices
    filter_coefficient_t filter_coefficients_numer;
    filter_coefficient_t filter_coefficients_denom;

    // Kalman gain
    state_vector_t kalman_gain;

    // Transition and measurement noise matrices
    transition_matrix_t transition_covariance;
    scalar_t measurement_covariance;

    // Fading memory alpha squared
    scalar_t fading_memory_alphasq;

    // df_error
    scalar_t df_error;

};

#endif
