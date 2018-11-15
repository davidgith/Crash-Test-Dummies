/**
 * @file "pses_ucbridge/Communication/syntax.h"
 * @brief Header file for the Syntax struct.
 *
*/

#ifndef SYNTAX_H
#define SYNTAX_H

#include <string>
#include <unordered_map>
#include <unordered_set>

/**
 * @struct Syntax syntax.h
 * @brief The Syntax class serves as a data struct to provide
 * information about important symbols for the communication classes.
 * (e.g. end_of_line, etc..)
 *
*/
struct Syntax
{
  std::string endOfMessage; /**< Special symbol that signals the end of a message. */
  std::string endOfFrame; /**< (optional) Signals the end of a line.  */
  std::string textMsgPrefix; /**< Signals an incoming plain text message.  */
  std::string answerOnCmdPrefix; /**< Signals an incoming answer on a command.  */
  std::string channelGrpMsgPrefix; /**< Signals an incoming sensor group message.  */
  std::string cmdErrorPrefix; /**< Signals a command error. */
  std::string genErrorPrefix; /**< Signals a board or communication error. */
  std::string optionsPrefix; /**< Signals a modification to a standard command. */
  std::unordered_set<std::string> grpErrorsAscii; /**< List of possible ascii encoded group message error codes. */
  std::unordered_map<std::string, std::unordered_set<unsigned int> > grpErrorsBinary; /**< List of possible binary encoded group message error codes. */
};

#endif // SYNTAX_H
