#ifndef YAMCR_SRC_POST_PROCESSORS_PROPHESEE_EVENT_EXTRACTOR_HPP
#define YAMCR_SRC_POST_PROCESSORS_PROPHESEE_EVENT_EXTRACTOR_HPP
#include <filesystem>

namespace YAMCR::PostProcessors {
	void processPropheseeData(std::filesystem::path rawPath);

} // YAMCR::PostProcessors

#endif //YAMCR_SRC_POST_PROCESSORS_PROPHESEE_EVENT_EXTRACTOR_HPP