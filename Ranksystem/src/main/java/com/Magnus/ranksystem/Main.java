package com.Magnus.ranksystem;

import com.Magnus.ranksystem.manager.NametagManager;
import com.Magnus.ranksystem.manager.RankManager;
import org.bukkit.Bukkit;
import org.bukkit.plugin.java.JavaPlugin;

/**
 * Main plugin class. Sets up the rank system by initializing
 *  - RankManager (loads/saves ranks.yml)
 *  - NametagManager (handles scoreboard-based nametags)
 * Registers the /rank command and a join listener.
 */
public class Main extends JavaPlugin {

    private RankManager rankManager;
    private NametagManager nametagManager;

    @Override
    public void onEnable() {
        getCommand("rank").setExecutor(new RankCommand(this));

        rankManager = new RankManager(this);
        nametagManager = new NametagManager(this);

        Bukkit.getPluginManager().registerEvents(new RankListener(this), this);
    }

    public RankManager getRankManager() {return rankManager;}
    public NametagManager getNametagManager() {return nametagManager;}

}
