package org.firstinspires.ftc.teamcode.lib.vision.common

import org.opencv.core.Mat

interface ImgDebuggable {
    // Available debug options: each option key maps to a Boolean (enabled/disabled)
    val availableDbgOptions: MutableMap<String, Boolean>

    // Debug data stored at this node.
    val dbgData: MutableMap<String, Mat>

    // Child nodes in the debug tree.
    val dbgChildren: MutableList<ImgDebuggable>

    /**
     * Sets the debug option for the given key.
     */
    fun setDbgOption(key: String, enabled: Boolean) {
        availableDbgOptions[key] = enabled
    }

    /**
     * Convenience method: sets all options in the given list to the given enabled state.
     */
    fun setDbgOptions(options: List<String>, enabled: Boolean) {
        options.forEach { key -> availableDbgOptions[key] = enabled }
    }


    /**
     * Checks if a given debug option is enabled.
     */
    fun isDbgOptionEnabled(key: String): Boolean = availableDbgOptions.getOrDefault(key, false)

    /**
     * Adds a debug entry (intermediate Mat) under the specified key, only if the option is enabled.
     */
    fun addDbgEntry(key: String, mat: Mat) {
        if (isDbgOptionEnabled(key)) {
            dbgData[key] = mat
        }
    }

    /**
     * Recursively collects debug data from this node and its children,
     * including only entries whose option is enabled.
     */
    fun getAllDbgData(): Map<String, Mat> {
        val result = mutableMapOf<String, Mat>()
        for ((key, enabled) in availableDbgOptions) {
            if (enabled && dbgData.containsKey(key)) {
                result[key] = dbgData[key]!!
            }
        }
        for (child in dbgChildren) {
            result.putAll(child.getAllDbgData())
        }
        return result
    }

    /**
     * Adds a child debuggable component.
     */
    fun addDbgChild(child: ImgDebuggable) {
        dbgChildren.add(child)
    }

    fun addDbgChildren(children: List<ImgDebuggable>) {
        dbgChildren.addAll(children)
    }
}


open class DefaultImgDebuggable : ImgDebuggable {
    override val availableDbgOptions: MutableMap<String, Boolean> = mutableMapOf()
    override val dbgData: MutableMap<String, Mat> = mutableMapOf()
    override val dbgChildren: MutableList<ImgDebuggable> = mutableListOf()
}

class RootImgDebuggable(vararg children: ImgDebuggable): DefaultImgDebuggable(){
    init {
        children.forEach { addDbgChild(it) }
    }
}