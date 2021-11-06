#pragma once

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

namespace grasp_detect
{
    namespace util
    {
        /**
         *
         * \brief Configuration file
         *
         * Reads parameters from a configuration file (`*.cfg`). The configuration file
         * is a key-value storage.
         *
         */
        class ConfigFile
        {
        public:
            /**
             * \brief Constructor.
             * \param fName the location of the configuration file
             */
            ConfigFile(const std::string &fName);

            /**
   * \brief Extract all keys.
   * \return `false` if the configuration file cannot be found, `true` otherwise
   */
            bool ExtractKeys();

            /**
   * \brief Check if a key exists.
   * \param key the key
   * \return `false` if the configuration file cannot be found, `true` otherwise
   */
            bool keyExists(const std::string &key) const;

            /**
   * \brief Return the value at a given key.
   * \param key the key associated to the value
   * \param defaultValue default value of the given key
   * \return the value associated to the key
   */
            template <typename ValueType>
            ValueType getValueOfKey(const std::string &key,
                                    ValueType const &defaultValue) const
            {
                if (!keyExists(key))
                    return defaultValue;

                return string_to_T<ValueType>(contents.find(key)->second);
            }

            /**
   * \brief Return the value at a given key as a `string`.
   * \param key the key
   * \param defaultValue default value of the given key
   * \return the value as a `string`
   */
            std::string getValueOfKeyAsString(const std::string &key,
                                              const std::string &defaultValue);

            /**
   * \brief Return the value at a given key as a `std::vector<double>`.
   * \param key the key
   * \param defaultValue default value of the given key
   * \return the value as a `std::vector<double>`
   */
            std::vector<double> getValueOfKeyAsStdVectorDouble(
                const std::string &key, const std::string &defaultValue);

            /**
   * \brief Return the value at a given key as a `std::vector<int>`.
   * \param key the key
   * \param defaultValue default value of the given key
   * \return the value as a `std::vector<int>`
   */
            std::vector<int> getValueOfKeyAsStdVectorInt(const std::string &key,
                                                         const std::string &defaultValue);

            /**
   * \brief Convert value of type `T` to `string`.
   * \param value the value to be converted
   * \return the value as a `string`
   */
            template <typename T>
            std::string T_to_string(T const &val) const
            {
                std::ostringstream ostr;
                ostr << val;

                return ostr.str();
            }

            /**
   * \brief Convert value of type `string` to type `T`.
   * \param value the value to be converted
   * \return the value as type `T`
   */
            template <typename T>
            T string_to_T(std::string const &val) const
            {
                std::istringstream istr(val);
                T returnVal;
                if (!(istr >> returnVal))
                    std::cout << "CFG: Not a valid " + (std::string) typeid(T).name() +
                                     " received!\n";

                return returnVal;
            }

        private:
            void removeComment(std::string &line) const;

            bool onlyWhitespace(const std::string &line) const;

            bool validLine(const std::string &line) const;

            void extractKey(std::string &key, size_t const &sepPos,
                            const std::string &line) const;

            void extractValue(std::string &value, size_t const &sepPos,
                              const std::string &line) const;

            void extractContents(const std::string &line);

            void parseLine(const std::string &line, size_t const lineNo);

            std::vector<double> stringToDouble(const std::string &str);

            std::vector<int> stringToInt(const std::string &str);

            std::map<std::string, std::string> contents;
            std::string fName;
        };

    } // namespace util
} // namespace grasp_detect
