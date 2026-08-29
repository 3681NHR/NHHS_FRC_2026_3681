package frc.utils

import java.util.LinkedList

abstract class Periodic {

    init {
        registeredClasses.add(this)
    }

    abstract fun update()

    companion object {
        private val registeredClasses: LinkedList<Periodic> = LinkedList()

        @JvmStatic
        fun updateAll() {
            for (p in registeredClasses) {
                p.update()
            }
        }
    }
}
