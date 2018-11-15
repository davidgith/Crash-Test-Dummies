/**
 * @file "pses_ucbridge/Communication/parameter.h"
 * @brief Header file for the Parameter::Parameter, Parameter::GenericParameter
 *and Parameter::ParameterMap
 *class.
 *
*/

#ifndef PARAMETER_H
#define PARAMETER_H

#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <utility>
#include <boost/algorithm/string.hpp>

/**
 * @namespace Parameter
 * @brief Classes in this Namespace implement dynamic type functionality.
 *
*/
namespace Parameter
{

/**
 * @class Parameter::Parameter parameter.h
 * @brief The Parameter::Parameter class is the abstract base class of all
 *Classes containing dynamically typed attributes.
 *
 * This class is a workaround allowing c++ Lists/Arrays/etc. to contain multiple
 *dynamic types.
 *
*/
class Parameter
{
public:
  /**
   * @brief Parameter::Parameter constructor.
   *
   * Name must be unique within a ParameterMap object.
   * Allowed parameter types: 'uint8_t', 'int8_t', 'uint16_t', 'int16_t',
   *'uint32_t', 'int32_t', 'uint64_t', 'int64_t', 'float32_t', 'float64_t',
   *'string_t', 'string_t[]'
   * @param[in] name unique identifier of the parameter
   * @param[in] type type of the parameter
  */
  Parameter(const std::string& name, const std::string& type)
      : name(name), type(type)
  {
    isParamValid = true;
  }
  /**
   * @brief Parameter::Parameter destructor.
  */
  virtual ~Parameter() {}
  /**
   * @brief Get the name of this parameter.
   * @return name of this parameter
  */
  const std::string& getName() const { return name; }
  /**
   * @brief Get the type of this parameter.
   * @return type of this parameter
  */
  const std::string& getType() const { return type; }
  /**
   * @brief Get the byte size of this parameter.
   * @return byte size of this parameter
  */
  virtual const int getTypeByteSize() const = 0;
  /**
   * @brief Get the sign of this parameter.
   * @return sign of this parameter, If (sign(parameter)== +) -> 1, Else -1.
  */
  virtual const bool isTypeSigned() const = 0;
  /**
   * @brief Is this parameter an arithmetic type?
   * @return true if type equals signed/unsigned char/short/int/long etc. or
   * float/double
  */
  virtual const bool isTypeArithmetic() const = 0;
  /**
   * @brief Defines whether this paramters value is NaN/invalid or not.
   * @param[in] isParamValid sets this parameter value to valid/invalid
  */
  void setValid(const bool isParamValid) { this->isParamValid = isParamValid; }
  /**
   * @brief Is this parameters value NaN/invalid?
   * @return true if parameter value is not NaN/invalid, else false.
  */
  const bool isValid() const { return isParamValid; }

private:
  std::string name;  /**< Unique identifier a parameter. */
  std::string type;  /**< Type of a parameter. */
  bool isParamValid; /**< Is this parameters value NaN/invalid? */
};

/**
 * @class Parameter::GenericParameter parameter.h
 * @brief The Parameter::GenericParameter class is a sub class of the
 *Parameter::Parameter class.
 *
 * This class is a workaround allowing c++ Lists/Arrays/etc. to contain multiple
 *dynamic types.
 *
*/
template <typename T> class GenericParameter : public Parameter
{
public:
  /**
   * @brief Parameter::GenericParameter constructor.
   *
   * Name must be unique within a ParameterMap object.
   * Allowed parameter types: 'uint8_t', 'int8_t', 'uint16_t', 'int16_t',
   *'uint32_t', 'int32_t', 'uint64_t', 'int64_t', 'float32_t', 'float64_t',
   *'string_t', 'string_t[]'
   * @param[in] name unique identifier of the parameter
   * @param[in] type type of the parameter
  */
  GenericParameter(const std::string& name, const std::string& type)
      : Parameter(name, type)
  {
  }
  /**
   * @brief Get the byte size of this parameter.
   * @return byte size of this parameter
  */
  const int getTypeByteSize() const { return sizeof(m_data); }
  /**
   * @brief Get the sign of this parameter.
   * @return sign of this parameter, If (sign(parameter)== +) -> 1, Else -1.
  */
  const bool isTypeSigned() const { return std::is_signed<T>(); }
  /**
   * @brief Is this parameter an arithmetic type?
   * @return true if type equals signed/unsigned char/short/int/long etc. or
   * float/double
  */
  const bool isTypeArithmetic() const { return std::is_arithmetic<T>(); }
  /**
   * @brief Set the value this paramter contains.
   * @param[in] m_data value to be set
  */
  void setData(const T& m_data) { this->m_data = m_data; }
  /**
   * @brief Get the value this paramter contains.
   * @return value of this parameter
  */
  const T& getData() const { return m_data; }

private:
  T m_data; /**< Value of this parameter. */
};

/**
 * @class Parameter::ParameterMap parameter.h
 * @brief The Parameter::ParameterMap class provides a map functionality for
 *Parameter::Parameter objects containing data of different types
 *
*/
class ParameterMap
{
public:
  /**
   * @brief Parameter::ParameterMap constructor.
  */
  ParameterMap()
  {
    parameters = std::unordered_map<std::string, std::shared_ptr<Parameter>>();
  }
  /**
   * @brief Is a parameter with the given name contained within this map?.
   * @param[in] name unique name of a paramter
   * @return true if parameter is contained within this map, else false.
  */
  const bool isParamInMap(const std::string& name) const
  {
    return parameters.find(name) != parameters.end();
  }
  /**
   * @brief Get the amount of parameters contained within this map.
   * @return number of parameters in this map
  */
  const int size() const { return parameters.size(); }
  /**
   * @brief Get the string representation of all parameters contained within
   * this map.
   * @return string of all parameters in this map
  */
  std::string toString() const
  {
    std::stringstream ss = std::stringstream();
    for (auto item : parameters)
    {
      ss << "Name: " << item.first << ", Type: " << item.second->getType()
         << "\n";
    }
    ss << "List size: " << size();
    return ss.str();
  }

  /**
   * @brief Insert a new parameter in this map, with the given name, type and
   *value.
   *
   * Name must be unique within a ParameterMap object.
   * Allowed parameter types: 'uint8_t', 'int8_t', 'uint16_t', 'int16_t',
   *'uint32_t', 'int32_t', 'uint64_t', 'int64_t', 'float32_t', 'float64_t',
   *'string_t', 'string_t[]'
   * @param[in] name unique identifier of a parameter in this map
   * @param[in] type type of a parameter in this map
   * @param[in] value value of a parameter in this map
   * @param[in] isValid is the value of the inserted paramter valid? (Default: true)
   * @throws std::exception
  */
  template <typename T>
  void insertParameter(const std::string& name, const std::string& type,
                       const T& value, const bool isValid = true)
  {
    std::shared_ptr<Parameter> param =
        std::shared_ptr<Parameter>(new GenericParameter<T>(name, type));
    param->setValid(isValid);
    std::dynamic_pointer_cast<GenericParameter<T>>(param)->setData(value);
    parameters.insert(std::make_pair(name, param));
  }
  /**
   * @brief Get the value of a parameter in this map with the given name.
   * @param[in] name unique identifier of a parameter in this map
   * @param[out] value value of a parameter in this map
   * @throws std::exception
  */
  template <typename T>
  void getParameterValue(const std::string& name, T& value) const
  {
    if (parameters.find(name) == parameters.end())
      throw std::out_of_range("Key: \"" + name + "\" not in Map!");
    if (sizeof(value) > parameters.at(name)->getTypeByteSize())
      throw std::invalid_argument("Key: \"" + name + "\" with Type: \"" +
                                  parameters.at(name)->getType() +
                                  "\" doesn't match given variable type.");
    value = std::dynamic_pointer_cast<GenericParameter<T>>(parameters.at(name))
                ->getData();
  }
  /**
   * @brief Get the dynamic typed parameter contained in this map with the given name.
   * @param[in] name unique identifier of a parameter in this map
   * @return the dynamic typed paramter
  */
  template <typename T>
  const std::shared_ptr<GenericParameter<T>>&
  getDynamicParameter(const std::string& name) const
  {
    if (parameters.find(name) == parameters.end())
      throw std::out_of_range("Key: \"" + name + "\" not in Map!");
    return std::dynamic_pointer_cast<GenericParameter<T>>(parameters.at(name));
  }
  const std::shared_ptr<Parameter>& getParameter(const std::string& name) const
  {
    if (parameters.find(name) == parameters.end())
      throw std::out_of_range("Key: \"" + name + "\" not in Map!");
    return parameters.at(name);
  }
  /**
   * @brief Get the string representation of a value from a parameter in this map with the given name.
   * @param[in] name unique identifier of a parameter in this map
   * @param[out] out string representation of a value from a parameter in this map
   * @throws std::exception
  */
  void getParameterValueAsString(const std::string& name,
                                 std::string& out) const
  {
    if (parameters.find(name) == parameters.end())
      throw std::out_of_range("Key: \"" + name + "\" not in Map!");
    const std::string& type = parameters.at(name)->getType();
    int size = getParameter(name)->getTypeByteSize();
    bool isSigned = getParameter(name)->isTypeSigned();
    bool isArithmetic = getParameter(name)->isTypeArithmetic();
    if (type.compare("int8_t") == 0)
    {
      if (size > 1 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      char value;
      getParameterValue(name, value);
      out = std::to_string(static_cast<int>(value));
    }
    else if (type.compare("uint8_t") == 0)
    {
      if (size > 1 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      unsigned char value;
      getParameterValue(name, value);
      out = std::to_string(static_cast<unsigned int>(value));
    }
    else if (type.compare("int16_t") == 0)
    {
      if (size > 2 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      short value;
      getParameterValue(name, value);
      out = std::to_string(static_cast<int>(value));
    }
    else if (type.compare("uint16_t") == 0)
    {
      if (size > 2 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      unsigned short value;
      getParameterValue(name, value);
      out = std::to_string(static_cast<unsigned int>(value));
    }
    else if (type.compare("int32_t") == 0)
    {
      if (size > 4 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      int value;
      getParameterValue(name, value);
      out = std::to_string(value);
    }
    else if (type.compare("uint32_t") == 0)
    {
      if (size > 4 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      unsigned int value;
      getParameterValue(name, value);
      out = std::to_string(value);
    }
    else if (type.compare("int64_t") == 0)
    {
      if (size > 8 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      long value;
      getParameterValue(name, value);
      out = std::to_string(value);
    }
    else if (type.compare("uint64_t") == 0)
    {
      if (size > 8 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      unsigned long value;
      getParameterValue(name, value);
      out = std::to_string(value);
    }
    else if (type.compare("float32_t") == 0)
    {
      if (size > 4 || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      float value;
      getParameterValue(name, value);
      out = std::to_string(value);
    }
    else if (type.compare("float64_t") == 0)
    {
      if (size > 8 || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      double value;
      getParameterValue(name, value);
      out = std::to_string(value);
    }
    else if (type.compare("string_t") == 0)
    {
      // wie string type mismatch bestimmen ?
      if (isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      getParameterValue(name, out);
    }
    else if (type.compare("string_t[]") == 0)
    {
      if (isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      // wie string type mismatch bestimmen ?
      std::vector<std::string> sArray;
      getParameterValue(name, sArray);
      std::stringstream ss = std::stringstream();
      for (std::string s : sArray)
      {
        ss << " " << s;
      }
      out = ss.str().substr(1, std::string::npos);
    }
    else
    {
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" is an unsupported type!");
    }
  }
  /**
   * @brief Set the value of a parameter in this map with the given name from a string.
   * @param[in] name unique identifier of a parameter in this map
   * @param[in] input value to be set encoded as ascii string
   * @param[in] isValid is the given value valid? (Default: true)
   * @throws std::exception
  */
  void setParameterValueAsString(const std::string& name,
                                 const std::string& input,
                                 const bool isValid = true)
  {
    if (parameters.find(name) == parameters.end())
      throw std::out_of_range("Key: \"" + name + "\" not in Map!");
    parameters[name]->setValid(isValid);
    if (!isValid)
    {
      return;
    }
    const std::string& type = parameters.at(name)->getType();
    int size = getParameter(name)->getTypeByteSize();
    bool isSigned = getParameter(name)->isTypeSigned();
    bool isArithmetic = getParameter(name)->isTypeArithmetic();
    if (type.compare("int8_t") == 0)
    {
      if (size > 1 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      int value = std::stoi(input);
      std::dynamic_pointer_cast<GenericParameter<char>>(parameters[name])
          ->setData(static_cast<char>(value));
    }
    else if (type.compare("uint8_t") == 0)
    {
      if (size > 1 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      unsigned long value = std::stoul(input);
      std::dynamic_pointer_cast<GenericParameter<unsigned char>>(
          parameters[name])->setData(static_cast<unsigned char>(value));
    }
    else if (type.compare("int16_t") == 0)
    {
      if (size > 2 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      int value = std::stoi(input);
      std::dynamic_pointer_cast<GenericParameter<short>>(parameters[name])
          ->setData(static_cast<short>(value));
    }
    else if (type.compare("uint16_t") == 0)
    {
      if (size > 2 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      unsigned long value = std::stoul(input);
      std::dynamic_pointer_cast<GenericParameter<unsigned short>>(
          parameters[name])->setData(static_cast<unsigned short>(value));
    }
    else if (type.compare("int32_t") == 0)
    {
      if (size > 4 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      int value = std::stoi(input);
      std::dynamic_pointer_cast<GenericParameter<int>>(parameters[name])
          ->setData(value);
    }
    else if (type.compare("uint32_t") == 0)
    {
      if (size > 4 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      unsigned long value = std::stoul(input);
      std::dynamic_pointer_cast<GenericParameter<unsigned int>>(
          parameters[name])->setData(value);
    }
    else if (type.compare("int64_t") == 0)
    {
      if (size > 8 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      long value = std::stol(input);
      std::dynamic_pointer_cast<GenericParameter<long long>>(parameters[name])
          ->setData(value);
    }
    else if (type.compare("uint64_t") == 0)
    {
      if (size > 8 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      unsigned long value = std::stoul(input);
      std::dynamic_pointer_cast<GenericParameter<unsigned long long>>(
          parameters[name])->setData(value);
    }
    else if (type.compare("float32_t") == 0)
    {
      if (size > 4 || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      float value = std::stof(input);
      std::dynamic_pointer_cast<GenericParameter<float>>(parameters[name])
          ->setData(value);
    }
    else if (type.compare("float64_t") == 0)
    {
      if (size > 8 || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      double value = std::stod(input);
      std::dynamic_pointer_cast<GenericParameter<double>>(parameters[name])
          ->setData(value);
    }
    else if (type.compare("string_t") == 0)
    {
      if (isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      std::dynamic_pointer_cast<GenericParameter<std::string>>(parameters[name])
          ->setData(input);
    }
    else if (type.compare("string_t[]") == 0)
    {
      if (isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      std::vector<std::string> split;
      boost::split(split, input, boost::is_any_of(" "));

      std::dynamic_pointer_cast<GenericParameter<std::vector<std::string>>>(
          parameters[name])->setData(split);
    }
    else
    {
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" is an unsupported type!");
    }
  }
  /**
   * @brief Insert a new parameter in this map, with the given name and type.
   *
   * Name must be unique within a ParameterMap object.
   *
   * Allowed parameter types: 'uint8_t', 'int8_t', 'uint16_t', 'int16_t',
   *'uint32_t', 'int32_t', 'uint64_t', 'int64_t', 'float32_t', 'float64_t',
   *'string_t', 'string_t[]'
   *
   * Parameter is set to a default value of 0.
   *
   * @param[in] name unique identifier of a parameter in this map
   * @param[in] type type of a parameter in this map
   * @param[in] isValid is the value of the inserted paramter valid? (Default: true)
   * @throws std::exception
  */
  void insertParameter(const std::string& name, const std::string& type,
                       const bool isValid = true)
  {
    std::shared_ptr<Parameter> param;

    if (type.compare("int8_t") == 0)
    {
      char value = 0;
      param =
          std::shared_ptr<Parameter>(new GenericParameter<char>(name, type));
      std::dynamic_pointer_cast<GenericParameter<char>>(param)->setData(value);
    }
    else if (type.compare("uint8_t") == 0)
    {
      unsigned char value = 0;
      param = std::shared_ptr<Parameter>(
          new GenericParameter<unsigned char>(name, type));
      std::dynamic_pointer_cast<GenericParameter<unsigned char>>(param)
          ->setData(value);
    }
    else if (type.compare("int16_t") == 0)
    {
      short value = 0;
      param =
          std::shared_ptr<Parameter>(new GenericParameter<short>(name, type));
      std::dynamic_pointer_cast<GenericParameter<short>>(param)->setData(value);
    }
    else if (type.compare("uint16_t") == 0)
    {
      unsigned short value = 0;
      param = std::shared_ptr<Parameter>(
          new GenericParameter<unsigned short>(name, type));
      std::dynamic_pointer_cast<GenericParameter<unsigned short>>(param)
          ->setData(value);
    }
    else if (type.compare("int32_t") == 0)
    {
      int value = 0;
      param = std::shared_ptr<Parameter>(new GenericParameter<int>(name, type));
      std::dynamic_pointer_cast<GenericParameter<int>>(param)->setData(value);
    }
    else if (type.compare("uint32_t") == 0)
    {
      unsigned int value = 0;
      param = std::shared_ptr<Parameter>(
          new GenericParameter<unsigned int>(name, type));
      std::dynamic_pointer_cast<GenericParameter<unsigned int>>(param)
          ->setData(value);
    }
    else if (type.compare("int64_t") == 0)
    {
      long value = 0;
      param =
          std::shared_ptr<Parameter>(new GenericParameter<long>(name, type));
      std::dynamic_pointer_cast<GenericParameter<long>>(param)->setData(value);
    }
    else if (type.compare("uint64_t") == 0)
    {
      unsigned long value = 0;
      param = std::shared_ptr<Parameter>(
          new GenericParameter<unsigned long>(name, type));
      std::dynamic_pointer_cast<GenericParameter<unsigned long>>(param)
          ->setData(value);
    }
    else if (type.compare("float32_t") == 0)
    {
      float value = 0.0;
      param =
          std::shared_ptr<Parameter>(new GenericParameter<float>(name, type));
      std::dynamic_pointer_cast<GenericParameter<float>>(param)->setData(value);
    }
    else if (type.compare("float64_t") == 0)
    {
      double value = 0.0;
      param =
          std::shared_ptr<Parameter>(new GenericParameter<double>(name, type));
      std::dynamic_pointer_cast<GenericParameter<double>>(param)
          ->setData(value);
    }
    else if (type.compare("string_t") == 0)
    {
      param = std::shared_ptr<Parameter>(
          new GenericParameter<std::string>(name, type));
      std::dynamic_pointer_cast<GenericParameter<std::string>>(param)
          ->setData("no value");
    }
    else if (type.compare("string_t[]") == 0)
    {
      std::vector<std::string> value;
      param = std::shared_ptr<Parameter>(
          new GenericParameter<std::vector<std::string>>(name, type));
      std::dynamic_pointer_cast<GenericParameter<std::vector<std::string>>>(
          param)->setData(value);
    }
    else
    {
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" is an unsupported type!");
    }
    param->setValid(isValid);

    parameters.insert(std::make_pair(name, param));
  }

private:
  std::unordered_map<std::string, std::shared_ptr<Parameter>> parameters; /**< Parameter map (first: parameter name, second:
                    Parameter::Parameter)*/
};
}
#endif // PARAMETER_H
