#include <vector>
#include <string>
#include <cnr_param/core/string.h>

namespace cnr
{
namespace param
{
namespace core
{

std::vector<std::string> tokenize(const std::string &str, const std::string &delim)
{
  std::vector<std::string> tokens;
  int tokenStart = 0;
  auto delimPos = str.find_first_of(delim);
  while (delimPos != std::string::npos)
  {
    std::string tok = str.substr(tokenStart, delimPos - tokenStart);
    tokens.push_back(tok);

    delimPos++;
    tokenStart = delimPos;
    delimPos = str.find_first_of(delim, delimPos);

    if (delimPos == std::string::npos)
    {
      std::string tok = str.substr(tokenStart, delimPos - tokenStart);
      tokens.push_back(tok);
    }
  }
  std::vector<std::string>::iterator it = tokens.begin();
  while (it != tokens.end())
  {
    if (it->empty())
    {
      it = tokens.erase(it);
    }
    else
    {
      ++it;
    }
  }
  return tokens;
}

}
}
}