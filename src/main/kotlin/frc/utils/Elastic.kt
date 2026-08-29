package frc.utils

import com.fasterxml.jackson.annotation.JsonProperty
import com.fasterxml.jackson.core.JsonProcessingException
import com.fasterxml.jackson.databind.ObjectMapper
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.PubSubOption
import edu.wpi.first.networktables.StringPublisher
import edu.wpi.first.networktables.StringTopic

object Elastic {
    private val notificationTopic: StringTopic =
        NetworkTableInstance.getDefault().getStringTopic("/Elastic/RobotNotifications")
    private val notificationPublisher: StringPublisher =
        notificationTopic.publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true))
    private val selectedTabTopic: StringTopic =
        NetworkTableInstance.getDefault().getStringTopic("/Elastic/SelectedTab")
    private val selectedTabPublisher: StringPublisher =
        selectedTabTopic.publish(PubSubOption.keepDuplicates(true))
    private val objectMapper = ObjectMapper()

    enum class NotificationLevel {
        INFO,
        WARNING,
        ERROR
    }

    @JvmStatic
    fun sendNotification(notification: Notification) {
        try {
            notificationPublisher.set(objectMapper.writeValueAsString(notification))
        } catch (e: JsonProcessingException) {
            e.printStackTrace()
        }
    }

    @JvmStatic
    fun selectTab(tabName: String) {
        selectedTabPublisher.set(tabName)
    }

    @JvmStatic
    fun selectTab(tabIndex: Int) {
        selectTab(tabIndex.toString())
    }

    class Notification {
        @JsonProperty("level")
        var level: NotificationLevel? = null

        @JsonProperty("title")
        var title: String? = null

        @JsonProperty("description")
        var description: String? = null

        @JsonProperty("displayTime")
        var displayTimeMillis: Int = 0

        @JsonProperty("width")
        var width: Double = 0.0

        @JsonProperty("height")
        var height: Double = 0.0

        constructor() : this(NotificationLevel.INFO, "", "")

        constructor(
            level: NotificationLevel,
            title: String,
            description: String,
            displayTimeMillis: Int,
            width: Double,
            height: Double
        ) {
            this.level = level
            this.title = title
            this.displayTimeMillis = displayTimeMillis
            this.description = description
            this.height = height
            this.width = width
        }

        constructor(level: NotificationLevel, title: String, description: String) :
            this(level, title, description, 3000, 350.0, -1.0)

        constructor(level: NotificationLevel, title: String, description: String, displayTimeMillis: Int) :
            this(level, title, description, displayTimeMillis, 350.0, -1.0)

        constructor(level: NotificationLevel, title: String, description: String, width: Double, height: Double) :
            this(level, title, description, 3000, width, height)

        fun setDisplayTimeSeconds(seconds: Double) {
            displayTimeMillis = Math.round(seconds * 1000).toInt()
        }

        fun withLevel(level: NotificationLevel): Notification {
            this.level = level
            return this
        }

        fun withTitle(title: String): Notification {
            this.title = title
            return this
        }

        fun withDescription(description: String): Notification {
            this.description = description
            return this
        }

        fun withDisplaySeconds(seconds: Double): Notification =
            withDisplayMilliseconds(Math.round(seconds * 1000).toInt())

        fun withDisplayMilliseconds(displayTimeMillis: Int): Notification {
            this.displayTimeMillis = displayTimeMillis
            return this
        }

        fun withWidth(width: Double): Notification {
            this.width = width
            return this
        }

        fun withHeight(height: Double): Notification {
            this.height = height
            return this
        }

        fun withAutomaticHeight(): Notification {
            this.height = -1.0
            return this
        }

        fun withNoAutoDismiss(): Notification {
            this.displayTimeMillis = 0
            return this
        }
    }
}
