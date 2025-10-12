#pragma once

#include <kfr/all.hpp>
#include <tsl/robin_map.h>
#include <array>
#include <string>
#include <string_view>

#include <data/interface.hpp>

namespace cortex::analysis
{
    class BatchAnalyzer
    {
    public:
        // ctor with configurable window size and overlap percentage
        explicit BatchAnalyzer(
            const data::EEGData& eeg_data,
            size_t window_size,
            double overlap_percentage = 75.0
        );

        // process all channels
        void process_all_channels();

        // Process a specific channel
        void process_channel(std::string_view channel_name);

        // Get the amplitude data for a specific band and channel
        [[nodiscard]] const kfr::univector<double>& get_band_amplitude(
            data::FrequencyBand band,
            std::string_view channel_name) const;

        // Get the dominant frequency band at a specific time
        [[nodiscard]] data::FrequencyBand get_dominant_band(
            std::string_view channel_name,
            size_t time_index) const;

        // Visualization information structure
        struct VisualizationInfo
        {
            double radius_multiplier;
            double transparency;
            data::FrequencyBand band;
        };

        // get visualization information for all bands at a specific time
        [[nodiscard]] std::array<VisualizationInfo, 5> get_visualization_info(
            std::string_view channel_name,
            size_t time_index) const;

        // get the maximum frame index available in the processed data
        [[nodiscard]] size_t get_max_frame_index() const;

        // conv time index to frame index
        [[nodiscard]] size_t time_index_to_frame(size_t time_index) const;

        [[nodiscard]] size_t get_window_size() const
        {
            return m_window_size;
        }

        // TODO: get radius multiplier for the main unprocessed data based on the intensity of amplitude

        [[nodiscard]] size_t get_hop_size() const
        {
            return m_hop_size;
        }

        [[nodiscard]] double get_frequency_resolution() const
        {
            return m_sampling_rate / m_window_size;
        }

        [[nodiscard]] double get_sampling_rate() const
        {
            return m_sampling_rate;
        }

        [[nodiscard]] const data::EEGData& get_eeg_data() const
        {
            return m_eeg_data;
        }

    private:
        const data::EEGData& m_eeg_data;
        double m_sampling_rate;

        // FFT params
        size_t m_window_size; // window size (power of 2)
        size_t m_hop_size; // hop size (for overlap)

        // struct to hold amplitude values for each frequency band
        struct BandAmplitudes
        {
            kfr::univector<double> delta;
            kfr::univector<double> theta;
            kfr::univector<double> alpha;
            kfr::univector<double> beta;
            kfr::univector<double> gamma;
        };

        // map of channel name to band amplitudes
        tsl::robin_map<std::string, BandAmplitudes> m_channel_amplitudes;

        // calc band amplitude from the power spectrum
        [[nodiscard]] static double calculate_band_amplitude(
            const kfr::univector<double>& power_spectrum,
            double freq_resolution,
            double min_freq,
            double max_freq);

        // calculate radius multiplier based on relative amplitude
        [[nodiscard]] static double calculate_radius_multiplier(double amplitude, double max_amplitude);

        // calculate transparency based on relative amplitude
        [[nodiscard]] static double calculate_transparency(double amplitude, double max_amplitude);

        [[nodiscard]] static size_t round_to_power_of_2(size_t value);
    };
} // namespace cortex::analysis
