package frc.utils

class AprilTagRegion private constructor(
    private val redTags: IntArray,
    private val blueTags: IntArray
) {
    private enum class Regions(val red: IntArray, val blue: IntArray) {
        STATION(intArrayOf(1, 2), intArrayOf(12, 13)),
        PROCESSOR(intArrayOf(3), intArrayOf(16)),
        BARGE(intArrayOf(4, 5), intArrayOf(14, 15)),
        REEF(intArrayOf(6, 7, 8, 9, 10, 11), intArrayOf(17, 18, 19, 20, 21, 22)),
        EMPTY(intArrayOf(), intArrayOf())
    }

    private constructor(region: Regions) : this(region.red, region.blue)

    fun red(): IntArray = redTags

    fun blue(): IntArray = blueTags

    fun both(): IntArray {
        val both = IntArray(redTags.size + blueTags.size)
        System.arraycopy(redTags, 0, both, 0, redTags.size)
        System.arraycopy(blueTags, 0, both, redTags.size, blueTags.size)
        return both
    }

    fun and(other: AprilTagRegion): AprilTagRegion {
        val newRed = redTags.copyOf(redTags.size + other.redTags.size)
        System.arraycopy(other.redTags, 0, newRed, redTags.size, other.redTags.size)
        val newBlue = blueTags.copyOf(blueTags.size + other.blueTags.size)
        System.arraycopy(other.blueTags, 0, newBlue, blueTags.size, other.blueTags.size)
        return AprilTagRegion(newRed, newBlue)
    }

    companion object {
        @JvmField
        val kStation = AprilTagRegion(Regions.STATION)

        @JvmField
        val kProcessor = AprilTagRegion(Regions.PROCESSOR)

        @JvmField
        val kBarge = AprilTagRegion(Regions.BARGE)

        @JvmField
        val kReef = AprilTagRegion(Regions.REEF)

        @JvmField
        val kEmpty = AprilTagRegion(Regions.EMPTY)
    }
}
