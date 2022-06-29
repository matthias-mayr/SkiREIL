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
#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <cstdint>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <glog/logging.h>
#include <skireil/utils/json.hpp>

namespace skireil {

namespace parameters {

using nlohmann::json;

// Forward declarations
template <class T> class Param;
class ParamBase;
using ParamVec = std::vector<std::shared_ptr<skireil::parameters::ParamBase>>;

// Enum for parameter types
enum ParamType { Real, Integer, Ordinal, Categorical };
enum DataType { Int, Float, String };

void fatalError(const std::string &msg) {
  LOG(FATAL) << "FATAL: " << msg << std::endl;
}

std::ostream &operator<<(std::ostream &out, const ParamType &PT) {
  switch (PT) {
  case Real:
    out << "Real";
    break;
  case Integer:
    out << "Integer";
    break;
  case Ordinal:
    out << "Ordinal";
    break;
  case Categorical:
    out << "Categorical";
    break;
  }
  return out;
}

std::string getTypeAsString(const ParamType &PT) {
  std::string TypeString;
  switch (PT) {
  case Real:
    TypeString = "real";
    break;
  case Integer:
    TypeString = "integer";
    break;
  case Ordinal:
    TypeString = "ordinal";
    break;
  case Categorical:
    TypeString = "categorical";
    break;
  }
  return TypeString;
}

ParamType getTypeFromString(const std::string &TypeString) {
  if (TypeString == "real") {
    return Real;
  } else if (TypeString == "integer") {
    return Integer;
  } else if (TypeString == "ordinal") {
    return Ordinal;
  } else if (TypeString == "categorical") {
    return Categorical;
  } else {
    throw std::invalid_argument("Invalid parameter type: " + TypeString);
  }
}

class ParamBase {
private:
  std::string Name;
  std::string const Key;
  ParamType Type;
  DataType DType;

public:
  ParamBase(std::string _Name = "", ParamType _Type = ParamType::Integer,
                   DataType _DType = DataType::Int)
      : Name(_Name), Key(_Name), Type(_Type), DType(_DType)
         {}
  virtual ~ParamBase() = default;

  virtual std::shared_ptr<ParamBase> clone() const = 0;

  std::string getName() const { return Name; }
  void setName(std::string _Name) { Name = _Name; }

  ParamType getType() const { return Type; }
  void setType(ParamType _Type) { Type = _Type; }

  std::string getKey() const { return Key; }

  DataType getDType() const { return DType; }
  void setDType( DataType _DType) {DType = _DType;}

  bool operator==(const std::string &_Key) {
    if (Key == _Key) {
      return true;
    } else {
      return false;
    }
  }

  bool operator==(const ParamBase &IP) {
    if (Key == IP.getKey()) {
      return true;
    } else {
      return false;
    }
  }

  void print () {
    std::cout << getKey() << ":";
    std::cout << "\n  Name: " << getName();
    std::cout << "\n  Type: " << getType();
    print(std::cout);
  }

  virtual void print(std::ostream &out) const {}

  friend std::ostream &operator<<(std::ostream &out,
                                  const ParamBase &IP) {
    out << IP.getKey() << ":";
    out << "\n  Name: " << IP.getName();
    out << "\n  Type: " << IP.getType();
    IP.print(out);
    return out;
  }
};

// Parameter object
template <class T> class Param : public ParamBase {
private:
  std::vector<T> Range;
  T Value;

public:
  Param(std::string _Name = "", ParamType _Type = ParamType::Integer)
      : ParamBase(_Name, _Type) {
        if (std::is_same<T, int>::value)
          setDType(Int);
        else if (std::is_same<T, float>::value)
          setDType(Float);
        else if (std::is_same<T, std::string>::value)
          setDType(String);
        else
          fatalError("Unhandled data type used for input parameter. New data types can be added by augmenting the DataType enum, and modifying this constructor accordingly.");
      }

  virtual std::shared_ptr<ParamBase> clone() const {
      return std::make_shared<Param<T>>(*this);
  };

  void setRange(std::vector<T> const &_Range) { Range = _Range; }
  std::vector<T> getRange() const { return Range; }

  T getVal() const { return Value; }
  void setVal(T _Value) { Value = _Value; }

  bool operator==(const std::string &_Key) {
    if (getKey() == _Key) {
      return true;
    } else {
      return false;
    }
  }

  bool operator==(const Param<T> &IP) {
    if (getKey() == IP.getKey()) {
      return true;
    } else {
      return false;
    }
  }

  void print(std::ostream &out, bool one_line = false) const {
    std::string sep{"\n"};
    if (one_line) {
      sep = "\t";
    }
    if (getType() == ParamType::Ordinal ||
        getType() == ParamType::Categorical) {
      out << sep << "  Range: {";
      char separator[1] = "";
      for (auto i : getRange()) {
        out << separator << i;
        separator[0] = ',';
      }
      out << "}";
    } else if (getType() == ParamType::Integer ||
               getType() == ParamType::Real) {
      out << sep << "  Range: [";
      char separator[1] = "";
      for (auto i : getRange()) {
        out << separator << i;
        separator[0] = ',';
      }
      out << "]";
    }
  }

  friend std::ostream &operator<<(std::ostream &out,
                                  const Param<T> &IP) {
    out << IP.getKey() << ":";
    out << "\n  Name: " << IP.getName();
    out << "\n  Type: " << IP.getType();
    IP.print(out);
    return out;
  }
};

// Function that populates input parameters
// void collectInputParams(size_t dim, std::vector<ParamBase *> &InParams, bool has_boundary, float lbound, float ubound) {
//     for (size_t i = 0; i < dim; i++) {
//         Param<float> *param = new Param<float>("x"+std::to_string(i), ParamType::Real);
//         if (has_boundary) {
//             std::vector<float> floatRange = {lbound, ubound};
//         param->setRange(floatRange);
//         }
//         InParams.push_back(param);
//     }
// }

std::vector<std::shared_ptr<ParamBase>> cloneParamVec(const std::vector<std::shared_ptr<ParamBase>>& pv)
{
    std::vector<std::shared_ptr<ParamBase>> npv;
    npv.reserve(pv.size());
    for (auto const& fptr : pv)
        npv.emplace_back(fptr->clone());
    return npv;
}

// Function for mapping input parameter based on key
int findParamByKey(const ParamVec &InParams, std::string Key) {
    for (int it = 0; it < static_cast<int>(InParams.size()); ++it) {
        if (*(InParams[it]) == Key) {
            return it;
        }
    }
    return -1;
}

//Function that sets the input parameter value
void setInputValue(ParamBase *Parameter, std::string ParamVal) {
    switch(Parameter->getDType()) {
        case Int:
            static_cast<skireil::parameters::Param<int>*>(Parameter)->setVal(stoi(ParamVal));
            break;
        case Float:
            static_cast<skireil::parameters::Param<float>*>(Parameter)->setVal(stof(ParamVal));
            break;
        case String:
            static_cast<skireil::parameters::Param<std::string>*>(Parameter)->setVal(ParamVal);
            break;
    }
}

Eigen::VectorXd paramsToEigen(const std::vector<ParamBase *>& InParams) {
    Eigen::VectorXd v(InParams.size());
    for (size_t i = 0; i < InParams.size(); i++) {
        switch(InParams[i]->getDType()) {
            case Int:
                v[i] = static_cast<double>(static_cast<Param<int>*>(InParams[i])->getVal());
                break;
            case Float:
                v[i] = static_cast<double>(static_cast<Param<float>*>(InParams[i])->getVal());
                break;
            default:
                fatalError("Unsupported parameter type for Eigen conversion.");
        }
    }
    return v;
}

std::vector<std::shared_ptr<ParamBase>> paramVecFromJson(const json& j) {
  using json = nlohmann::json;
  std::vector<std::shared_ptr<ParamBase>> sp;
  for (const auto& el : j.items()) {
    std::cout << el.key() << " : " << el.value() << "\n";
    std::cout << "ParamType: " << getTypeFromString(el.value()["parameter_type"].get<std::string>()) << std::endl;
    ParamType pt {getTypeFromString(el.value()["parameter_type"].get<std::string>())};
    json v = el.value()["values"];
    switch (pt) {
      case Real: {
        auto p = std::make_shared<Param<float>>(el.key(), Real);
        CHECK_EQ(v.size(),2) << "real values must have an upper and lower bound";
        p->setRange({v[0].get<float>(), v[1].get<float>()});
        if (el.value().contains("parameter_default")) {
          p->setVal(el.value()["parameter_default"].get<float>());
        } else {
          p->setVal(0.0);
        }
        sp.push_back(p);
        break;
      }
      case Categorical: {
        auto p = std::make_shared<Param<std::string>>(el.key(), Categorical);
        CHECK_GE(v.size(),2) << "Categorical values must two or more elements";
        std::vector<std::string> range;
        for (const auto& vel : v) {
          range.push_back(vel.get<std::string>());
        }
        p->setRange(range);
        if (el.value().contains("parameter_default")) {
          p->setVal(el.value()["parameter_default"].get<std::string>());
        }
        sp.push_back(p);
        break;
      }
      case Integer: {
        auto p = std::make_shared<Param<int>>(el.key(), Integer);
        CHECK_EQ(v.size(),2) << "integer values must have an upper and lower bound";
        p->setRange({v[0].get<int>(), v[1].get<int>()});
        if (el.value().contains("parameter_default")) {
          p->setVal(el.value()["parameter_default"].get<int>());
        } else {
          p->setVal(0);
        }
        sp.push_back(p);
        break;
      }
      case Ordinal: {
        auto p = std::make_shared<Param<float>>(el.key(), Ordinal);
        CHECK_GE(v.size(),2) << "ordinal values must two or more elements";
        std::vector<float> range;
        for (const auto& vel : v) {
          range.push_back(vel.get<float>());
        }
        p->setRange(range);
        if (el.value().contains("parameter_default")) {
          p->setVal(el.value()["parameter_default"].get<float>());
        } else {
          p->setVal(0.0);
        }
        sp.push_back(p);
        break;
      }
     }
  }
  return sp;
}

json fileToJson(std::string filename) {
    std::ifstream i(filename);
    json j;
    if (!i.good()) {
        throw std::runtime_error("Error when trying to read " + filename);
    }
    i >> j;
    return j;
}

json paramVecToJson(const ParamVec& pv) {
  json ps;
  for (const auto& InParam : pv) {
    json HMParam;
    HMParam["parameter_type"] = getTypeAsString(InParam->getType());
    switch (InParam->getDType()) {
    case Int: {
        auto p = static_cast<Param<int>*>(InParam.get());
        HMParam["values"] = json(p->getRange()); 
        HMParam["parameter_default"] = p->getVal();
        break;
    }
    case Float: {
        auto p = static_cast<Param<float>*>(InParam.get());
        HMParam["values"] = json(p->getRange()); 
        HMParam["parameter_default"] = p->getVal();
        break;
    }
    case String: {
        auto p = static_cast<Param<std::string>*>(InParam.get());
        HMParam["values"] = json(p->getRange()); 
        HMParam["parameter_default"] = p->getVal();
        break;
    }
    }
    ps[InParam->getKey()] = HMParam;
  }
  return ps;
}

bool paramVecToFile(const ParamVec& pv, std::string filename) {
  std::ofstream o(filename);
  o << std::setw(4) << paramVecToJson(pv) << std::endl;
  o.close();
  return true;
}

std::ostream &operator<<(std::ostream &out, const std::vector<std::shared_ptr<ParamBase>> &pv) {
  for (const auto& p : pv) {
    out << p.get();
  }
  return out;
}

std::stringstream paramVecToStream(const std::vector<std::shared_ptr<ParamBase>>& pv, bool one_line = false, bool verbose = false) {
  std::stringstream ss;
  std::string sep{"\n"};
  if (one_line) {
    sep = "\t";
  }
  for (const auto& p : pv) {
    ss << p->getKey() << ":" << sep << "  Name: " << p->getName() << sep;
    switch (p->getDType()) {
    case Int:
        ss << "  Value: " << static_cast<Param<int>*>(p.get())->getVal() << sep; 
        break;
    case Float:
        ss << "  Value: " << static_cast<Param<float>*>(p.get())->getVal() << sep; 
        break;
    case String:
        ss << "  Value: " << static_cast<Param<std::string>*>(p.get())->getVal() << sep; 
    }    
    if (verbose) {
      ss << "  Type: " << p->getType() << sep;
      switch (p->getDType()) {
      case Int: {
          ss << "  Range: ";
          static_cast<Param<int>*>(p.get())->print(ss, one_line);
          ss << sep; 
          break;
      }
      case Float: {
          ss << "  Range: ";
          static_cast<Param<float>*>(p.get())->print(ss, one_line);
          ss << sep; 
          break;
      }
      case String: {
          ss << "  Range: ";
          static_cast<Param<std::string>*>(p.get())->print(ss, one_line);
          ss << sep;
      }
      }
    }
  }
  return ss;
}

} // namespace parameters
} // namespace skireil

#endif
