//|
//|    Copyright Inria July 2017
//|    This project has received funding from the European Research Council (ERC) under
//|    the European Union's Horizon 2020 research and innovation programme (grant
//|    agreement No 637972) - see http://www.resibots.eu
//|
//|    Contributor(s):
//|      - Matthias Mayr (matthias.mayr@cs.lth.se)
//|      - Konstantinos Chatzilygeroudis (konstantinos.chatzilygeroudis@inria.fr)
//|      - Rituraj Kaushik (rituraj.kaushik@inria.fr)
//|      - Roberto Rama (bertoski@gmail.com)
//|
//|
//|    This software is governed by the CeCILL-C license under French law and
//|    abiding by the rules of distribution of free software.  You can  use,
//|    modify and/ or redistribute the software under the terms of the CeCILL-C
//|    license as circulated by CEA, CNRS and INRIA at the following URL
//|    "http://www.cecill.info".
//|
//|    As a counterpart to the access to the source code and  rights to copy,
//|    modify and redistribute granted by the license, users are provided only
//|    with a limited warranty  and the software's author,  the holder of the
//|    economic rights,  and the successive licensors  have only  limited
//|    liability.
//|
//|    In this respect, the user's attention is drawn to the risks associated
//|    with loading,  using,  modifying and/or developing or reproducing the
//|    software by the user in light of its specific status of free software,
//|    that may mean  that it is complicated to manipulate,  and  that  also
//|    therefore means  that it is reserved for developers  and  experienced
//|    professionals having in-depth computer knowledge. Users are therefore
//|    encouraged to load and test the software's suitability as regards their
//|    requirements in conditions enabling the security of their systems and/or
//|    data to be ensured and,  more generally, to use and operate it in the
//|    same conditions as regards security.
//|
//|    The fact that you are presently reading this means that you have had
//|    knowledge of the CeCILL-C license and that you accept its terms.
//|
#ifndef SKIREL_MOTION_GENERATOR_CONFIGURATION
#define SKIREL_MOTION_GENERATOR_CONFIGURATION

#include <exception>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace motion_generator_configuration {
Eigen::VectorXd std_vec_to_Eigen(std::vector<double>& v) {
    double* ptr = &v[0];
    return Eigen::Map<Eigen::VectorXd>(ptr, v.size());
}

enum overlay_motions{
    NONE = 0,
    ARCHIMEDES = 1
};

inline std::string motion_to_string(const overlay_motions& m) {
    switch (m) {
        case overlay_motions::NONE: return "none";
        case overlay_motions::ARCHIMEDES: return "archimedes";
        default: throw std::invalid_argument("Invalid motion configuration for string conversion");
    }
};

inline overlay_motions string_to_motion(const std::string& s) {
    if (s == "none" || s.empty()) {
        return overlay_motions::NONE;
    } else if (s == "archimedes") {
        return overlay_motions::ARCHIMEDES;
    } else {
        throw std::invalid_argument("Invalid motion configuration from string conversion: " + s);
    }
};

struct overlay_conf {
    static const int size = 8;

    void from_eigen_vector(const Eigen::VectorXd& v) {
        if (v.size() != this->size) {
            throw std::runtime_error("Overlay Configuration has the wrong size. " + v.size());
        }
        size_t it {0};
        set_from_vec(_overlay_d, v, it);
        _overlay_motion = static_cast<overlay_motions>(std::round(v[it]));
        it++;
        _radius = v[it];
        it++;
        _path_distance = v[it];
        it++;
        _path_velocity = v[it];
        it++;
        _allow_decrease = static_cast<bool>(std::round(v[it]));
        it++;

        if (it != this->size) {
            throw std::range_error("Error in implemention when reading from vector.");
        }
    }

    Eigen::VectorXd to_eigen_vector() {
        Eigen::VectorXd ret(overlay_conf::size);
        ret << _overlay_d, static_cast<double>(_overlay_motion), _radius, _path_distance, _path_velocity, static_cast<double>(_allow_decrease);
        return ret;
    }

    friend bool operator==(const overlay_conf& lhs, const overlay_conf& rhs){
        if (lhs._overlay_d == rhs._overlay_d && lhs._overlay_motion == rhs._overlay_motion && lhs._radius == rhs._radius && lhs._path_distance == rhs._path_distance && lhs._path_velocity == rhs._path_velocity && lhs._allow_decrease == rhs._allow_decrease) {
            return true;
        } else {
            return false;
        }
    }
    friend bool operator!=(const overlay_conf& lhs, const overlay_conf& rhs){ return !(lhs == rhs); }

    bool check_validity() {
        if (_radius < 0.0 || _path_distance < 0.0 || _path_velocity < 0.0) {
            return false;
        }
        return true;
    }

    Eigen::Vector3d _overlay_d {Eigen::Vector3d::Zero()};
    overlay_motions _overlay_motion{overlay_motions::NONE};
    double _radius{0};
    double _path_distance{0};
    double _path_velocity{0};
    bool _allow_decrease{false};
private:
    template <typename T>
    void set_from_vec(T& el, const Eigen::VectorXd& v, size_t& it) {
        el << v.segment(it, el.size());
        it += el.size();
    }
};

struct mg_conf {
    static const int size = 21 + overlay_conf::size;

    mg_conf() = default;
    mg_conf(const Eigen::VectorXd& v) {
        from_eigen_vector(v);
    }

    Eigen::VectorXd to_eigen_vector() {
        Eigen::VectorXd ret(mg_conf::size);
        ret << _pos, _rot.coeffs(), _cart_stiffness, _nullspace_stiffness, _force, _torque, _overlay.to_eigen_vector(), _tool;
        return ret;
    };

    void from_eigen_vector(const Eigen::VectorXd& v) {
        if (v.size() != mg_conf::size) {
            throw std::runtime_error("Motion Generator Configuration has the wrong size. " + v.size());
        }
        size_t it {0};
        set_from_vec(_pos, v, it);
        _rot.coeffs() << v.segment(it, 4);
        _rot.normalize();
        it += 4;
        set_from_vec(_cart_stiffness, v, it);
        _nullspace_stiffness = v[it];
        it ++;
        set_from_vec(_force, v, it);
        set_from_vec(_torque, v, it);
        _overlay.from_eigen_vector(v.segment(it, overlay_conf::size));
        it += overlay_conf::size;
        _tool = v[it];
        it++;
        if (it != this->size) {
            throw std::range_error("Error in implemention when reading from vector.");
        }
        if (!check_validity()) {
            throw std::runtime_error("Motion configuration is invalid.");
        }
    }

    Eigen::Matrix<double, 7, 1> get_stiffnesses() const {
        Eigen::Matrix<double, 7, 1> s;
        s << _cart_stiffness, _nullspace_stiffness;
        return s;
    }

    Eigen::Matrix<double, 6, 1> get_wrench() const {
        Eigen::Matrix<double, 6, 1> w;
        w << _force, _torque;
        return w;
    }

    friend bool operator==(const mg_conf& lhs, const mg_conf& rhs){
        if (lhs._pos == rhs._pos && lhs._rot.coeffs() == rhs._rot.coeffs() && lhs._cart_stiffness == rhs._cart_stiffness && lhs._nullspace_stiffness == rhs._nullspace_stiffness && lhs._overlay == rhs._overlay && lhs._tool == rhs._tool) {
            return true;
        } else {
            return false;
        }
    }
    friend bool operator!=(const mg_conf& lhs, const mg_conf& rhs){ return !(lhs == rhs); }

    bool pose_equal(const mg_conf& other) const {
        if (_pos == other._pos && _rot.coeffs() == other._rot.coeffs()) {
            return true;
        } else {
            return false;
        }
    }

    bool check_validity() {
        bool invalid = (_cart_stiffness.array() < 0.0).any();
        if (invalid) {
            return false;
        }
        if (_nullspace_stiffness < 0.0) {
            return false;
        }
        return _overlay.check_validity();
    }

    Eigen::Vector3d _pos {Eigen::Vector3d::Zero()};
    Eigen::Quaterniond _rot {Eigen::Quaterniond::Identity()};

    Eigen::Matrix<double, 6, 1> _cart_stiffness {Eigen::Matrix<double, 6, 1>::Zero()};
    double _nullspace_stiffness {0};

    Eigen::Vector3d _force {Eigen::Vector3d::Zero()};
    Eigen::Vector3d _torque {Eigen::Vector3d::Zero()};

    overlay_conf _overlay;
    double _tool{-1};

private:
    template <typename T>
    void set_from_vec(T& el, const Eigen::VectorXd& v, size_t& it) {
        el << v.segment(it, el.size());
        it += el.size();
    }
};

const int mg_conf::size;
const int overlay_conf::size;

} // namespace motion_generator_configuration

#endif