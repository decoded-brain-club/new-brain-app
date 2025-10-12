#pragma once

#include <kfr/all.hpp>
#include <data/interface.hpp>

namespace cortex::data::filtering
{
    /**
	* @brief Sets the initial reference for EEG data
	*
	* EEG electrodes measure relative voltage, requiring a reference to interpret
	* the signals correctly. This function subtracts a reference signal (computed
	* from specified channels) from all other electrodes.
	*
	* @param data The EEG data to be referenced
	* @param referenceChannels Vector of channel names to use as reference
	* @return A new EEGData object with referenced data
	*/

    inline EEGData set_initial_reference(const EEGData& data, const std::vector<std::string>& referenceChannels)
	{
		const size_t sampleCount = data.get_sample_count();
		if (sampleCount == 0)
		{
			return data;
		}

		// validate if they even  exist
		for (const auto& refChannel : referenceChannels)
		{
			try
			{
				std::ignore = data.get_channel(refChannel);
			}
			catch (const std::out_of_range&)
			{
				throw std::invalid_argument(fmt::format("Reference channel not found: {}", refChannel));
			}
		}

		// average of chosen reference channels at each timepoint
		kfr::univector referenceSignal(sampleCount, 0.0);

		for (size_t t = 0; t < sampleCount; ++t)
		{
			double sum = 0.0;
			for (const auto& refChannel : referenceChannels)
			{
				sum += data.get_channel(refChannel)[t];
			}
			referenceSignal[t] = sum / referenceChannels.size();
		}

		// subtract reference from all channels
		EEGData referencedData = data;
		for (const auto& channel : data.get_channel_names())
		{
			kfr::univector<double> channelData = data.get_channel(channel);
			channelData = channelData - referenceSignal;
			referencedData.set_channel(channel, std::move(channelData));
		}

		return referencedData;
	}


    /**
    * @brief Applies a high-pass filter to EEG data
    *
    * Removes slow drifts and DC offsets while preserving slower cortical potentials.
    * Uses an FIR filter with linear-phase properties for optimal signal preservation.
    *
    * @param data The EEG data to be filtered
    * @param cutoffFreq The cutoff frequency in Hz (typically 0.1-1 Hz)
    * @param filterOrder The order of the filter (must be odd for high-pass)
    * @return A new EEGData object with filtered data
    */
    inline EEGData apply_high_pass_filter(const EEGData& data, const double cutoffFreq, int filterOrder)
	{
		EEGData filteredData = data;

		// filter order is odd for symmetry
		if (filterOrder % 2 == 0)
		{
			filterOrder += 1;
		}

		const double samplingRate = data.m_samplingRate;
		const double normalizedCutoff = cutoffFreq / (samplingRate / 2.0);

		kfr::univector<double> taps(filterOrder);
		const auto kaiser = kfr::to_handle(kfr::window_kaiser<double>(taps.size(), 3.0));

		kfr::fir_highpass(taps, static_cast<kfr::identity<double>>(normalizedCutoff), kaiser, true);

		kfr::filter_fir<double> filter(taps);

		for (const auto& channel : data.get_channel_names())
		{
			kfr::univector<double> channelData = data.get_channel(channel);
			filter.apply(channelData);
			filteredData.set_channel(channel, std::move(channelData));
		}

		return filteredData;
	}

    /**
     * @brief Applies a low-pass filter to EEG data
     *
     * Eliminates high-frequency noise and muscle artifacts.
     * Typically used with cutoffs of 30-50 Hz for cognitive studies.
     *
     * @param data The EEG data to be filtered
     * @param cutoffFreq The cutoff frequency in Hz (typically 30-50 Hz)
     * @param filterOrder The order of the filter
     * @return A new EEGData object with filtered data
    */
    inline EEGData apply_lowpass_filter(const EEGData& data, const double cutoffFreq, const int filterOrder)
	{
		EEGData filteredData = data;

		const double samplingRate = data.m_samplingRate;
		const double normalizedCutoff = cutoffFreq / (samplingRate / 2.0);

		kfr::univector<double> taps(filterOrder);
		const auto kaiser = kfr::to_handle(kfr::window_kaiser<double>(taps.size(), 3.0));

		kfr::fir_lowpass(taps, normalizedCutoff, kaiser, true);

		kfr::filter_fir<double> filter(taps);

		for (const auto& channel : data.get_channel_names())
		{
			kfr::univector<double> channelData = data.get_channel(channel);
			filter.apply(channelData);
			filteredData.set_channel(channel, std::move(channelData));
		}

		return filteredData;
	}

    /**
    * @brief Applies a notch filter to EEG data
    *
    * Suppresses power-line interference (50 or 60 Hz) using an FIR notch filter.
    *
    * @param data The EEG data to be filtered
    * @param notchFreq The frequency to remove (typically 50 or 60 Hz)
    * @param bandwidth The width of the notch in Hz (typically 2-4 Hz)
    * @param filterOrder The order of the filter (must be odd)
    * @return A new EEGData object with filtered data
    */
    inline EEGData apply_notch_filter(const EEGData& data, const double notchFreq, const double bandwidth, int filterOrder)
	{
		EEGData filteredData = data;

		// filter order is odd for symmetry
		if (filterOrder % 2 == 0)
		{
			filterOrder += 1;
		}

		const double samplingRate = data.m_samplingRate;

		const double normalizedCenter = notchFreq / (samplingRate / 2.0);
		const double normalizedWidth = bandwidth / (samplingRate / 2.0);
		const double f1 = normalizedCenter - normalizedWidth / 2.0;
		const double f2 = normalizedCenter + normalizedWidth / 2.0;

		kfr::univector<double> taps(filterOrder);
		const auto kaiser = kfr::to_handle(kfr::window_kaiser<double>(taps.size(), 4.0));

		// higher beta for steeper transition
		kfr::fir_bandstop(taps,
						  f1,
						  f2,
						  kaiser,
						  true);

		kfr::filter_fir<double> filter(taps);

		for (const auto& channel : data.get_channel_names())
		{
			kfr::univector<double> channelData = data.get_channel(channel);
			filter.apply(channelData);
			filteredData.set_channel(channel, std::move(channelData));
		}

		return filteredData;
	}

    /**
 	* @brief Applies Common Average Reference (CAR) to EEG data
 	*
 	* Improves signal quality by removing shared noise and eliminating dependency
 	* on a single reference. CAR is computed as the mean of all electrodes,
 	* which is then subtracted from each individual electrode's signal.
 	* Most effective for high-density EEG (>10 channels).
 	*
 	* @param data The EEG data to be re-referenced
 	* @return A new EEGData object with CAR applied
	*/
    inline EEGData apply_common_average_reference(const EEGData& data)
	{
		const size_t sampleCount = data.get_sample_count();
		if (sampleCount == 0)
		{
			return data;
		}

		const std::vector<std::string> channelNames = data.get_channel_names();
		if (channelNames.empty())
		{
			return data;
		}

		kfr::univector<double> averageSignal(sampleCount, 0.0);

		for (size_t t = 0; t < sampleCount; ++t)
		{
			double sum = 0.0;
			for (const auto& channel : channelNames)
			{
				sum += data.get_channel(channel)[t];
			}
			averageSignal[t] = sum / channelNames.size();
		}

		// subtract common average from each channel
		EEGData referencedData = data;
		for (const auto& channel : channelNames)
		{
			kfr::univector<double> channelData = data.get_channel(channel);
			channelData = channelData - averageSignal;
			referencedData.set_channel(channel, std::move(channelData));
		}

		return referencedData;
	}

    /**
 	* @brief Applies Min-Max normalization to each EEG channel
 	*
 	* Normalizes the amplitude scale of each channel to a standard range,
 	* typically [0, 1] or [-1, 1]. This ensures all channels have comparable scales,
 	* which can be important for certain analyses or machine learning algorithms.
 	*
 	* @param data The EEG data to be normalized
 	* @param normalizeToRange01 If true, normalizes to [0,1], otherwise normalizes to [-1,1]
 	* @return A new EEGData object with normalized data
	*/
    inline EEGData apply_min_max_normalization(const EEGData& data, bool normalizeToRange01)
	{
		const size_t sampleCount = data.get_sample_count();
		if (sampleCount == 0)
		{
			return data;
		}

		EEGData normalizedData = data;
		for (const auto& channel : data.get_channel_names())
		{
			const kfr::univector<double>& channelData = data.get_channel(channel);

			auto [minIt, maxIt] = std::ranges::minmax_element(channelData);
			double minVal = *minIt;
			double maxVal = *maxIt;
			double range = maxVal - minVal;

			kfr::univector<double> normalizedChannel(channelData.size());

			if (std::abs(range) < 1e-10)
			{
				// flat channel (avoid division by zero)
				normalizedChannel = kfr::univector<double>(channelData.size(), 0.0);
			}
			else
			{
				// nirmalize to [0,1]
				normalizedChannel = (channelData - minVal) / range;

				// optionally rescale to [-1,1]
				if (!normalizeToRange01)
				{
					normalizedChannel = normalizedChannel * 2.0 - 1.0;
				}
			}

			normalizedData.set_channel(channel, std::move(normalizedChannel));
		}

		return normalizedData;
	}


    /**
    * @brief Applies baseline correction to EEG data
    *
    * Corrects each channel by subtracting the mean signal from a predefined baseline period.
    * This removes constant offsets per channel and aligns all channels to a common baseline.
    * Typically used for epoched EEG or at the start of continuous recordings.
    *
    * @param data The EEG data to be baseline-corrected
    * @param baselineStartSample First sample index of the baseline period
    * @param baselineEndSample Last sample index of the baseline period (inclusive)
    * @return A new EEGData object with baseline-corrected data
    */
    inline EEGData apply_baselining(
		const EEGData& data,
		size_t baselineStartSample,
		size_t baselineEndSample)
	{

		const size_t sampleCount = data.get_sample_count();
		if (sampleCount == 0)
		{
			return data;
		}

		if (baselineStartSample >= sampleCount || baselineEndSample >= sampleCount ||
			baselineStartSample > baselineEndSample)
		{
			throw std::invalid_argument(fmt::format(
				"Invalid baseline range: [{}, {}] for data with {} samples",
				baselineStartSample, baselineEndSample, sampleCount));
		}

		EEGData baselinedData = data;
		for (const auto& channel : data.get_channel_names())
		{
			const kfr::univector<double>& channelData = data.get_channel(channel);

			// baseline mean for this channel
			double baselineSum = 0.0;
			size_t baselineCount = baselineEndSample - baselineStartSample + 1;

			for (size_t t = baselineStartSample; t <= baselineEndSample; ++t)
			{
				baselineSum += channelData[t];
			}

			double baselineMean = baselineSum / baselineCount;

			// baseline-corrected data
			kfr::univector<double> baselinedChannel = channelData - baselineMean;

			// update
			baselinedData.set_channel(channel, std::move(baselinedChannel));
		}

		return baselinedData;
	}

    /**
     * @brief Overloaded version of apply_baselining that accepts time values in seconds
     *
     * @param data The EEG data to be baseline-corrected
     * @param baselineStartTime Start time of baseline period in seconds
     * @param baselineEndTime End time of baseline period in seconds
     * @return A new EEGData object with baseline-corrected data
     */
    inline EEGData apply_baselining(
		const EEGData& data,
		const double baselineStartTime,
		const double baselineEndTime)
	{
		const auto baselineStartSample = static_cast<size_t>(baselineStartTime * data.m_samplingRate);
		const auto baselineEndSample = static_cast<size_t>(baselineEndTime * data.m_samplingRate);

		return apply_baselining(data, baselineStartSample, baselineEndSample);
	}
} // namespace cortex::data::filtering
