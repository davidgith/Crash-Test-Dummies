/**
 * @file "pses_ucbridge/Communication/command.h"
 * @brief Header file for the Command class.
 *
*/

#ifndef COMMAND_H
#define COMMAND_H

#include <string>
#include <unordered_set>
#include <unordered_map>

#include <utility>
#include <vector>
#include <pses_ucbridge/Communication/syntax.h>
#include <pses_ucbridge/Communication/parameter.h>
#include <boost/algorithm/string.hpp>
#include <boost/range/algorithm/remove_if.hpp>
#include <boost/algorithm/string/trim.hpp>

/**
 * @struct CommandParams command.h
 * @brief The CommandParams class serves as a data struct to configure
 * Command objects.
 *
 * An instance of this struct is required to instantiate
 * a Command object.
 *
*/
struct CommandParams
{
  std::string name;  /**< Unique identifier of a Command object. */
  bool cmdHasParams; /**< Does this Command have dynamic parameters? */
  std::vector<std::pair<std::string, std::string>>
      params; /**< List of dynamic parameters of this command/response (first:
                 name, second: type) */
  std::string cmd;      /**< Command itself, containing keywords and parameters
                           flagged with '$' */
  bool cmdHasResponse;  /**< Does this Command trigger a response? */
  bool respHasParams;   /**< Does the response have dynamic parameters? */
  std::string response; /**< Response itself, containing keywords and parameters
                           flagged with '$' */
};

/**
 * @struct CommandOptions command.h
 * @brief The CommandOptions class serves as a data struct to inform
 * Command objects about all modifiers that can be applied to them.
 *
 * A list of instance of this struct is required to instantiate
 * a Command object, this list may also be empty.
 *
*/
struct CommandOptions
{
  std::string optName; /**< Unique identifier of a CommandOptions struct. */
  bool optHasParams;   /**< Does this CommandOption have dynamic parameters? */
  std::vector<std::pair<std::string, std::string>>
      params; /**< List of dynamic parameters of this option/response (first:
                 name, second: type) */
  std::string opt;       /**< Option itself, containing keywords and parameters
                            flagged with '$' */
  bool optReturnsParams; /**< Does the response have dynamic parameters? */
  std::string
      response;        /**< Response itself, containing keywords and parameters
                          flagged with '$'. NOTE: At the moment there may only be
                          one returned parameter per option!*/
  bool addsRespToGrps; /**< Does this option modify the response on a "set
                          group" command? NOTE: At the moment this option is
                          unused, hence untested, use with caution! */
};

/**
 * @class Command command.h
 * @brief The Command class builds the syntactic template for a specific
 * command defined in a CommandParams struct.
 *
 * A Command object builds its command message given the required parameter
 *values.
 * It can also verfy if a given response is valid, given the input parameters of
 *the
 * previously sent command.
 *
 * If a non empty CommandOptions list was given on initilization, a command
 *object can
 * change its command template based on required options.
 *
*/
class Command
{
  /**
   * @typedef void (Command::*insertInstruction)(const int&,
                                             const Parameter::ParameterMap&,
                                             std::string& out)
   * @brief Command template building function pointer type.
   * @param[in] index Index of the token in a cmdToken list.
   * @param[in] input Pointer to a ParameterMap object.
   * @param[out] out Output string, containing the requested token.
  */
  typedef void (Command::*insertInstruction)(const int&,
                                             const Parameter::ParameterMap&,
                                             std::string& out);

public:
  /**
   * @brief Command default constructor.
  */
  Command();
  /**
   * @brief Command constructor.
   * @param[in] cmdParams CommandParams struct that is used to configure a
   * Command object.
   * @param[in] cmdResponsePrefix Marks the beginning of a response.
   * @param[in] options Map of possible options, that may modify this command.
   *                 (first: name, second: CommandOptions)
   * @param[in] optionsPrefix Marks the appearance of an option
   *                             within a command template.
   * @throws std::exception
  */

  Command(const CommandParams& cmdParams, const std::shared_ptr<Syntax> syntax,
          const std::unordered_map<std::string,
                                   std::shared_ptr<CommandOptions>>& options);
  /**
   * @brief Generates a command string from the internal template given all
   * necessary parameter values.
   * @param[in] inputParams Input parameter map, containing all necessary
   * parameter values and types to create a command.
   * @param[out] out Generated command string.
   * @throws std::exception
  */
  void generateCommand(const Parameter::ParameterMap& inputParams,
                       std::string& out);
  /**
   * @brief Generates a command string from the internal template given all
   * necessary parameter values and options.
   * This method is able to take an options list as an additional argument to
   * modify its internal command template.
   * @param[in] inputParams Input parameter map, containing all necessary
   * parameter values and types to create a command, including parameter values
   * required by
   * given options.
   * @param[in] options List of options, to be applied to a command.
   * @param[out] out Generated command string.
   * @throws std::exception
  */
  void generateCommand(const Parameter::ParameterMap& inputParams,
                       const std::vector<std::string>& options,
                       std::string& out);
  /**
   * @brief Verifies a given response based on the internal template given all
   * necessary input parameter values.
   * @param[in] inputParams Input parameter map, containing all necessary
   * parameter values and types to verify a response.
   * @param[in] responseOrig Response on command to be verified.
   * @param[out] outputParams If the command requested returned parameters,
   * those will be put here.
   * @return true if response is valid, else false.
   * @throws std::exception
  */
  const bool verifyResponse(const Parameter::ParameterMap& inputParams,
                            const std::string& responseOrig,
                            Parameter::ParameterMap& outputParams);
  /**
   * @brief Verifies a given response based on the internal template given all
   * necessary input parameter values and options.
   * This method is able to take an options list as an additional argument to
   * verify a modified command template.
   * @param[in] inputParams Input parameter map, containing all necessary
   * parameter values and types to verify a response, including parameter values
   * required by
   * given options.
   * @param[in] options List of options, which have been applied to a command.
   * @param[in] responseOrig Response on command to be verified.
   * @param[out] outputParams If the command or command options requested
   * returned parameters, those will be put here.
   * @return true if response is valid, else false.
   * @throws std::exception
  */
  const bool verifyResponse(const Parameter::ParameterMap& inputParams,
                            const std::vector<std::string>& options,
                            const std::string& responseOrig,
                            Parameter::ParameterMap& outputParams);
  /**
   * @brief Getter for the unique name attribute.
   * @return name attribute value
  */
  const std::string& getName() const;

private:
  std::string name;    /**< Unique identifier of a Command object.*/
  bool cmdHasParams;   /**< Does this Command have dynamic parameters? */
  bool cmdHasResponse; /**< Does this Command trigger a response? */
  bool respHasParams;  /**< Does the response have dynamic parameters? */
  std::shared_ptr<Syntax> syntax; /**< Pointer to a Syntax object */
  std::unordered_map<std::string, std::shared_ptr<CommandOptions>> options; /**<
                       Map of possible options, that may modify this command.
                       (first: name, second: CommandOptions) */
  std::unordered_map<std::string, std::string> parameterTypes; /**<
                  Map of possible parameters, that may appear in this command.
                  (first: name, second: type) */
  std::unordered_set<std::string>
      cmdParameterSet; /**< Set of possible command parameters) */
  std::vector<std::string>
      cmdKeyWords; /**< List of command key words, ordered by
                      appearance in the template.) */
  std::vector<std::string> cmdParameter; /**< List of command parameters,
                                            ordered by appearance in the
                                            template.) */
  std::vector<std::pair<int, insertInstruction>>
      commandTemplate; /**< Command building template
                           (first: appearance index,
                           second: building function))
                           */

  std::string simpleResponse; /**< Static part of the response. */
  // string contains a keyWord if bool=false, else string contains paramName
  std::vector<std::pair<std::string, bool>>
      responseTemplate; /**< Response
               validation template:
               first: contains a keyWord if
               second:=false, else first
               contains paramName
               */
  /**
   * @brief Command template building function for a keyword token.
   * @param[in] index Index of the keyword in the cmdKeyWords list.
   * @param[in] input Pointer to a ParameterMap object, unused in this function.
   * @param[out] out Output string, containing the keyword.
  */
  void insertCmdKeyword(const int& index, const Parameter::ParameterMap& input,
                        std::string& out);
  /**
   * @brief Command template building function for a parameter token.
   * @param[in] index Index of the parameter in the cmdParameter list.
   * @param[in] input Pointer to a ParameterMap object, used to get the required
   * parameter
   * value.
   * @param[out] out Output string, containing the parameter value as string.
  */
  void insertCmdParameter(const int& index,
                          const Parameter::ParameterMap& input,
                          std::string& out);
};

#endif // COMMAND_H
