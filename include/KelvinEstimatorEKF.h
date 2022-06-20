#ifndef KELVINESTIMATOREKF_H
#define KELVINESTIMATOREKF_H
#include <Eigen/Dense>

template <typename scalar_t, int filter_order=2>
class KelvinEstimatorEKF {

public:
    typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> matrix_t;

    // State vectors
    typedef Eigen::Matrix<scalar_t, filter_order+3, 1> state_vector_t;

    // Covariance matrix
    typedef Eigen::Matrix<scalar_t, filter_order+3, filter_order+3> state_covariance_t;

    // Filter matrices
    typedef Eigen::Matrix<scalar_t, filter_order+1, 1> filter_coefficient_t;
    typedef Eigen::Matrix<scalar_t, filter_order, filter_order> filter_transition_matrix_t;
    typedef Eigen::Matrix<scalar_t, filter_order, 1> filter_input_matrix_t;
    typedef Eigen::Matrix<scalar_t, 1, filter_order> filter_output_matrix_t;
    typedef Eigen::Matrix<scalar_t, 1, 1> filter_feedthrough_matrix_t;

    // Combined system matrices
    typedef Eigen::Matrix<scalar_t, filter_order+3, filter_order+3> transition_matrix_t;
    typedef Eigen::Matrix<scalar_t, 1, filter_order+3> output_matrix_t;


    KelvinEstimatorEKF() {
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
        set_measurement_noise(1);

        // Initialize df_error
        df_error = 0;

        // Reset fading memory alpha
        set_fading_memory_alpha(1.0);

        last_voltage = 0;
    }

    void reset_filter() {
        // Set filter to feedthrough only.
        filter_transition_matrix.setZero();
        filter_input_matrix.setZero();
        filter_output_matrix.setZero();
        filter_feedthrough_matrix(0, 0) = 1;
    }

    void setup_filter(const filter_coefficient_t& b, const filter_coefficient_t& a) {
        // Create filter matrices from filter coefficients b and a.
        // NOTE: Implementation requires a(0) == 1.
        // See also: <https://www.dsprelated.com/freebooks/filters/Converting_State_Space_Form_Hand.html>
        filter_transition_matrix.setZero();
        filter_transition_matrix.template bottomLeftCorner<filter_order-1, filter_order-1>().setIdentity();
        filter_transition_matrix.row(0) = -a.template tail<filter_order>();

        filter_input_matrix.setZero();
        filter_input_matrix(0) = 1;

        filter_output_matrix = (
            b.template tail<filter_order>() - b(0) * a.template tail<filter_order>()
        );

        filter_feedthrough_matrix(0, 0) = b(0);
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

        typedef Eigen::Matrix<scalar_t, 3, 1> kelvin_state_vector_t;

        // Kelvin system
        auto f_kelvin = [](const kelvin_state_vector_t &x, scalar_t u) -> kelvin_state_vector_t {
            return x;
        };

        auto h_kelvin = [](const kelvin_state_vector_t &x, scalar_t u) -> scalar_t {
            auto delta = u - x[1];
            return x(0) - x(2) * delta * delta;
        };

        // Kelvin system Jacobians
        auto fjac_kelvin = [](const kelvin_state_vector_t &x, scalar_t u) {
            return Eigen::Matrix<scalar_t, 3, 3>::Identity();
        };

        auto hjac_kelvin = [](const kelvin_state_vector_t &x, scalar_t u) {
            auto delta = u - x[1];
            Eigen::Matrix<scalar_t, 1, 3> hjac;
            hjac(0) = 1;
            hjac(1) = 2 * x(2) * delta;
            hjac(2) = -delta * delta;
            return hjac;
        };

        // Combined system
        auto f = [&](const state_vector_t &x, scalar_t u) -> state_vector_t {
            state_vector_t xdot;
            auto kelvin_state = x.template head<3>();
            auto filter_state = x.template tail<filter_order>();
            xdot.template head<3>() = f_kelvin(kelvin_state, u);
            xdot.template tail<filter_order>() = (
                filter_transition_matrix * filter_state
                + filter_input_matrix * h_kelvin(kelvin_state, u)
            );
            return xdot;
        };

        auto h = [&](const state_vector_t &x, scalar_t u) -> scalar_t {
            auto kelvin_state = x.template head<3>();
            auto filter_state = x.template tail<filter_order>();
            auto tmp = (
                filter_output_matrix * filter_state
                + filter_feedthrough_matrix * h_kelvin(kelvin_state, u)
            );
            return tmp(0, 0);
        };

        auto fjac = [&](const state_vector_t &x, scalar_t u) -> transition_matrix_t {
            auto kelvin_state = x.template head<3>();
            transition_matrix_t out;
            out.template topLeftCorner<3, 3>() = fjac_kelvin(kelvin_state, u);
            out.template topRightCorner<3, filter_order>().setZero();
            out.template bottomLeftCorner<filter_order, 3>() = filter_input_matrix * hjac_kelvin(kelvin_state, u);
            out.template bottomRightCorner<filter_order, filter_order>() = filter_transition_matrix;
            return out;
        };

        auto hjac = [&](const state_vector_t &x, scalar_t u) -> output_matrix_t {
            auto kelvin_state = x.template head<3>();
            output_matrix_t out;
            out.template leftCols<3>() = filter_feedthrough_matrix * hjac_kelvin(kelvin_state, u);
            out.template rightCols<filter_order>() = filter_output_matrix;
            return out;
        };

        // Prediction (a priori state and covariance estimate)
        state_vector_t state_estimate_m = f(state_estimate, last_voltage);
        transition_matrix_t transition_matrix = fjac(state_estimate, last_voltage);

        state_covariance_t state_covariance_m = (
            fading_memory_alphasq
            * transition_matrix * state_covariance * transition_matrix.transpose()
            + transition_covariance
        );

        // Alternate method to prevent negative coefficient a.
        /*if (state_estimate_m(2) < 0) {
            state_estimate_m.template head<3>().setZero();
        }*/

        // Find innovation (measurement residual)
        scalar_t innovation = df - h(state_estimate_m, voltage);
        output_matrix_t output_matrix = hjac(state_estimate_m, voltage);
        scalar_t innovation_covariance = (
            output_matrix * state_covariance_m * output_matrix.transpose()
            + measurement_covariance
        );

        // Kalman gain
        kalman_gain = (
            state_covariance_m * output_matrix.transpose() / innovation_covariance
        );

        // Update (a posteriori state and covariance estimate)
        state_estimate = state_estimate_m + kalman_gain * innovation;
        transition_matrix_t tmp = transition_matrix_t::Identity() - kalman_gain * output_matrix;
        state_covariance = (
            tmp * state_covariance_m * tmp.transpose()
            + kalman_gain * measurement_covariance * kalman_gain.transpose()
        );

        // There can be multiple solutions for the quadratic characteristics.
        // When coefficient a < 0 is predicted, correct the state by projecting
        // the estimate to the allowed region. See eq. (25) in [1].
        // [1]: Simon, D. IET Control Theory & Applications 4, 1303–1318 (2010)
        const size_t coeffa_ind = 2;
        if (state_estimate(coeffa_ind) < 0) {
            state_estimate -= (
                state_covariance.col(coeffa_ind)
                / state_covariance(coeffa_ind, coeffa_ind)
                * state_estimate(coeffa_ind)
            );
            state_estimate(coeffa_ind) = 1e-9;
        }

        // Update df error using current state
        df_error = df - h(state_estimate, voltage);

        last_voltage = voltage;
    }

    matrix_t update_all(
        const Eigen::VectorXd& input_sample_vector, const Eigen::VectorXd& voltage_vector
    ) {
        int n = input_sample_vector.size();
        Eigen::Matrix<scalar_t, 4 + 6, Eigen::Dynamic> data;
        data.resize(4 + 6, n);

        for(int i = 0; i < n; i++){
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
    filter_transition_matrix_t filter_transition_matrix;
    filter_input_matrix_t filter_input_matrix;
    filter_output_matrix_t filter_output_matrix;
    filter_feedthrough_matrix_t filter_feedthrough_matrix;

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
