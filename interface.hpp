#pragma once

#include <string>
#include <vector>
#include <memory>
#include <stdexcept>

#include <fmt/format.h>
#include <tsl/robin_map.h>
#include <kfr/all.hpp>

namespace cortex::data
{
    enum class FrequencyBand
    {
        Delta, // 0.5-4 Hz
        Theta, // 4-8 Hz
        Alpha, // 8-13 Hz
        Beta, // 13-30 Hz
        Gamma // 30+ Hz
    };

    namespace FrequencyRange
    {
        static constexpr double DELTA_MIN = 0.5;
        static constexpr double DELTA_MAX = 4.0;
        static constexpr double THETA_MIN = 4.0;
        static constexpr double THETA_MAX = 8.0;
        static constexpr double ALPHA_MIN = 8.0;
        static constexpr double ALPHA_MAX = 13.0;
        static constexpr double BETA_MIN = 13.0;
        static constexpr double BETA_MAX = 30.0;
        static constexpr double GAMMA_MIN = 30.0;
        static constexpr double GAMMA_MAX = 100.0;
    }

    // Main EEG data container class
    class EEGData final
    {
    public:
        using ChannelData = tsl::robin_map<std::string, kfr::univector<double> >;

        double m_samplingRate = 128.0; // default sampling rate, Hz

        [[nodiscard]] const ChannelData& get_channels() const
        {
            return channels;
        }

        [[nodiscard]] const kfr::univector<double>& get_channel(const std::string_view channelName) const
        {
            const auto it = channels.find(channelName.data());
            if (it == channels.end())
            {
                throw std::out_of_range(fmt::format("Channel not found: {}", channelName));
            }
            return it->second;
        }

        void set_channel(const std::string_view channelName, kfr::univector<double> data)
        {
            channels[channelName.data()] = std::move(data);
        }

        [[nodiscard]] std::vector<std::string> get_channel_names() const
        {
            std::vector<std::string> names;
            names.reserve(channels.size());
            for (const auto& name : channels | std::views::keys)
            {
                names.push_back(name);
            }
            return names;
        }

        [[nodiscard]] size_t get_sample_count() const
        {
            if (channels.empty())
                return 0;
            return channels.begin()->second.size();
        }

    private:
        ChannelData channels;
    };

    // abstract interface for EEG data sources
    class EEGDataSource
    {
    public:
        EEGDataSource() = default;

        virtual ~EEGDataSource() = default;

        EEGDataSource(const EEGDataSource&) = delete;

        EEGDataSource& operator=(const EEGDataSource&) = delete;

        EEGDataSource(EEGDataSource&&) = default;

        EEGDataSource& operator=(EEGDataSource&&) = default;

        // check if data is available
        [[nodiscard]] virtual bool is_data_available() const = 0;

        // load/fetch data from the source
        virtual std::unique_ptr<EEGData> load_data() = 0;

        // open the data source
        virtual bool open() = 0;

        // close the data source
        virtual void close() = 0;

        // check if the source is open
        [[nodiscard]] virtual bool is_open() const = 0;

        // get the source name/identifier
        [[nodiscard]] virtual std::string get_source_name() const = 0;
    };
} // namespace data
