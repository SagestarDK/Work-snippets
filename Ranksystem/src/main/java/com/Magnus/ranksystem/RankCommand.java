package com.Magnus.ranksystem;

import org.bukkit.Bukkit;
import org.bukkit.ChatColor;
import org.bukkit.OfflinePlayer;
import org.bukkit.command.Command;
import org.bukkit.command.CommandExecutor;
import org.bukkit.command.CommandSender;
import org.bukkit.entity.Player;

public class RankCommand implements CommandExecutor {

    private Main main;

    public RankCommand(Main main) {
        this.main = main;
    }

    // /rank <player> <rank>

    @Override
    public boolean onCommand(CommandSender commandSender, Command command, String s, String[] args) {

        if (commandSender instanceof Player) {
            Player player = (Player) commandSender;

            if (player.isOp()) {
                if (args.length == 2) {
                    if (Bukkit.getOfflinePlayer(args[0]) != null) {
                        OfflinePlayer target = Bukkit.getOfflinePlayer(args[0]);

                        for (Rank rank : Rank.values()) {
                            if (rank.name().equalsIgnoreCase(args[1])) {
                                main.getRankManager().setRank(target.getUniqueId(), rank);

                                player.sendMessage(ChatColor.GREEN + "You changed " + target.getName() + "'s rank to " + rank.getDisplay());
                                if (target.isOnline()) {
                                    target.getPlayer().sendMessage(ChatColor.GREEN + player.getName() + " set your rank to " + rank.getDisplay());
                                }
                                return false;
                            }
                        }
                        player.sendMessage(ChatColor.RED + "You did not specify a valid rank! Options are Guest, Member, Admin & Owner.");
                    } else {
                        player.sendMessage(ChatColor.RED + "This user has never joined the server before!");
                    }
                } else {
                    player.sendMessage(ChatColor.RED + "Invalid usage! Please use /rank <player> <rank>.");
                }
            } else {
                player.sendMessage(ChatColor.RED + "You must be OP to use this command!");
            }
        }
        return false;
    }
}

