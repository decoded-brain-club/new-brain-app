#pragma once

#include <expected>
#include <simdjson.h>
#include <string>

#include <data/interface.hpp>

namespace cortex
{
	namespace data
	{
		// impl of EEGDataSource for JSON files
		class JSONFileSource final : public EEGDataSource
		{
		public:
			explicit JSONFileSource(std::string_view filePath);

			~JSONFileSource() override;

			// check if file exists and is readable
			[[nodiscard]] bool is_data_available() const override;

			// parse and load data from the JSON file
			std::unique_ptr<EEGData> load_data() override;

			// open the file
			bool open() override;

			// close the file
			void close() override;

			// check if file is open
			[[nodiscard]] bool is_open() const override;

			// get the source name (filename)
			[[nodiscard]] std::string get_source_name() const override;

			// set a new file path
			void set_file_path(std::string_view filePath);

			// get the current file path
			[[nodiscard]] const std::string& get_file_path() const;

		private:
			std::string m_filePath;
			bool m_fileOpen = false;
			simdjson::dom::parser m_parser_;


			enum class validation_error : uint8_t {
				NOT_OBJECT = 0,
				FIELD_NOT_ARRAY,
				EMPTY_ARRAY,
				VALUE_NOT_DOUBLE,
				METADATA_NOT_OBJECT
			};


			// Helper method to validate JSON structure
			static std::expected<bool, validation_error> validate_json_structure(const simdjson::dom::element& root);
		};
	} // namespace data
} // namespace cortex
